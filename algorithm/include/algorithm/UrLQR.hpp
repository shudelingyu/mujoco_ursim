#include <Eigen/Dense>
#include <qpOASES.hpp>
#include <iostream>
#include <chrono>
namespace algorithms
{
    class UR5LQRVibrationController
    {
    public:
        // 构造函数：初始化机械臂参数和LQR权重
        UR5LQRVibrationController(double control_freq = 1000.0)
            : dt_(1.0 / control_freq),
              last_torque_(Eigen::VectorXd::Zero(6))
        {
            // 初始化UR5动力学参数（示例值）
            M_ = Eigen::MatrixXd::Identity(6, 6) * 2.0;  // 质量矩阵
            C_ = Eigen::MatrixXd::Identity(6, 6) * 0.1;  // 阻尼矩阵
            K_ = Eigen::MatrixXd::Identity(6, 6) * 50.0; // 刚度矩阵

            // 构建扩展状态空间（18维 = 位置+速度+加速度）
            buildStateSpaceModel();

            // 设置LQR权重矩阵（振动抑制核心参数）
            Q_ = Eigen::MatrixXd::Zero(18, 18);
            Q_.block(0, 0, 6, 6) = 10.0 * Eigen::MatrixXd::Identity(6, 6);  // 位置权重
            Q_.block(6, 6, 6, 6) = 1.0 * Eigen::MatrixXd::Identity(6, 6);   // 速度权重
            Q_.block(12, 12, 6, 6) = 5.0 * Eigen::MatrixXd::Identity(6, 6); // 加速度权重（关键振动抑制项）

            R_ = 0.01 * Eigen::MatrixXd::Identity(6, 6); // 控制输入权重

            // 离线计算LQR增益矩阵
            K_lqr_ = solveLQR(A_, B_, Q_, R_);
        }

        // 主控制循环接口 (1kHz实时调用)
        Eigen::VectorXd computeControlTorque(
            const Eigen::VectorXd &q,
            const Eigen::VectorXd &dq,
            const Eigen::VectorXd &ddq,
            const Eigen::VectorXd &target_q)
        {
            // 1. 构建扩展状态向量 [q, dq, ddq]
            Eigen::VectorXd x(18);
            x << q, dq, ddq;

            // 2. 计算跟踪误差 (状态空间原点为平衡点)
            Eigen::VectorXd x_error = x;
            x_error.head(6) -= target_q; // 位置误差

            // 3. LQR最优控制律: u = -K * x_error
            Eigen::VectorXd tau = -K_lqr_ * x_error;

            // 4. 添加前馈补偿提升跟踪性能
            tau += K_ * (target_q - q) + C_ * (-dq);

            // 5. 执行器饱和保护
            torqueSaturation(tau, -100.0, 100.0); // ±100Nm限幅

            last_torque_ = tau;
            return tau;
        }

        // 获取当前LQR增益矩阵
        Eigen::MatrixXd getLQRGain() const
        {
            return K_lqr_;
        }

    private:
        // 构建状态空间模型[7](@ref)
        void buildStateSpaceModel()
        {
            A_ = Eigen::MatrixXd::Zero(18, 18);
            // 速度层: d(q)/dt = dq
            A_.block(0, 6, 6, 6) = Eigen::MatrixXd::Identity(6, 6);
            // 加速度层: d(dq)/dt = ddq
            A_.block(6, 12, 6, 6) = Eigen::MatrixXd::Identity(6, 6);
            // 动力学层: d(ddq)/dt = M⁻¹(τ - C·ddq - K·q)
            A_.block(12, 0, 6, 6) = -M_.inverse() * K_;
            A_.block(12, 6, 6, 6) = -M_.inverse() * C_;
            A_.block(12, 12, 6, 6) = -M_.inverse() * C_;

            B_ = Eigen::MatrixXd::Zero(18, 6);
            B_.block(12, 0, 6, 6) = M_.inverse(); // τ输入影响加速度变化率
        }

        // 求解Riccati方程获取LQR增益[4,7](@ref)
        Eigen::MatrixXd solveLQR(const Eigen::MatrixXd &A,
                                 const Eigen::MatrixXd &B,
                                 const Eigen::MatrixXd &Q,
                                 const Eigen::MatrixXd &R)
        {
            // 离散化系统（前向欧拉）
            Eigen::MatrixXd Ad = Eigen::MatrixXd::Identity(A.rows(), A.cols()) + A * dt_;
            Eigen::MatrixXd Bd = B * dt_;

            // 设置QP问题: min 1/2*xᵀHx + gᵀx
            int n = Ad.rows();
            int m = Bd.cols();

            // 构建Hessian矩阵
            Eigen::MatrixXd H = 2 * (Bd.transpose() * Q * Bd + R);
            // 构建梯度项
            Eigen::MatrixXd g = 2 * Bd.transpose() * Q * Ad;

            // 使用qpOASES求解[4](@ref)
            qpOASES::SQProblem solver(m, 0); // 0表示无等式约束
            qpOASES::Options options;
            options.setToMPC();
            options.printLevel = qpOASES::PL_NONE;
            solver.setOptions(options);

            // 转换为qpOASES数据类型
            qpOASES::real_t *H_qp = new qpOASES::real_t[m * m];
            qpOASES::real_t *g_qp = new qpOASES::real_t[m];
            Eigen::Map<Eigen::MatrixXd>(H_qp, m, m) = H;
            Eigen::Map<Eigen::VectorXd>(g_qp, m) = g;

            // 求解QP问题
            int nWSR = 100;
            solver.init(H_qp, g_qp, nullptr, nullptr, nullptr, nullptr, nWSR);

            // 提取最优控制增益
            qpOASES::real_t K_opt[m * n];
            solver.getPrimalSolution(K_opt);
            Eigen::MatrixXd K = Eigen::Map<Eigen::MatrixXd>(K_opt, m, n);

            delete[] H_qp;
            delete[] g_qp;

            return K;
        }

        // 执行器饱和保护
        void torqueSaturation(Eigen::VectorXd &tau, double min, double max)
        {
            for (int i = 0; i < tau.size(); ++i)
            {
                if (tau[i] > max)
                    tau[i] = max;
                if (tau[i] < min)
                    tau[i] = min;
            }
        }

        // 成员变量
        Eigen::MatrixXd M_, C_, K_;   // 动力学参数
        Eigen::MatrixXd A_, B_;       // 状态空间矩阵
        Eigen::MatrixXd Q_, R_;       // LQR权重矩阵
        Eigen::MatrixXd K_lqr_;       // LQR反馈增益
        Eigen::VectorXd last_torque_; // 上一周期力矩（平滑用）
        double dt_;                   // 控制周期
    };
}
// // 示例使用场景
// int main() {
//     // 1. 初始化控制器（1kHz控制频率）
//     UR5LQRVibrationController ctrl(1000.0);

//     // 2. 模拟实时控制循环
//     Eigen::VectorXd q(6), dq(6), ddq(6), target_q(6);
//     q << 0.1, -0.2, 0.3, 0.15, -0.1, 0.05;
//     dq.setZero();
//     ddq.setZero();
//     target_q.setZero();

//     // 模拟100个控制周期
//     for (int i = 0; i < 100; ++i) {
//         // 实际应用中应从硬件读取
//         Eigen::VectorXd tau = ctrl.computeControlTorque(q, dq, ddq, target_q);

//         // 输出力矩到执行器（此处简化为打印）
//         std::cout << "Cycle " << i << ": Torque = " << tau.transpose() << std::endl;

//         // 状态更新（此处省略实际动力学仿真）
//         // ...
//     }

//     return 0;
// }
#pragma once
#include <Eigen/Dense>
#include <qpOASES.hpp>
#include <vector>
namespace algorithms
{
    class UR5VibrationMPC
    {
    public:
        UR5VibrationMPC(int horizon, double dt, int delay_steps = 3);

        Eigen::VectorXd solve(const Eigen::VectorXd &current_state,
                              const Eigen::VectorXd &target_joint);

    private:
        void buildExtendedModel();

        void precomputeQPWeights(double Q_pos, double R_jerk, double S_torque);

        // 系统参数
        Eigen::MatrixXd M_, C_, K_, D_;   // 动力学参数
        Eigen::MatrixXd A_c_, A_d_, B_d_; // 连续/离散状态矩阵
        Eigen::MatrixXd S_, E_, Q_, H_;   // MPC预测矩阵
        qpOASES::SQProblem *solver_;
        std::vector<qpOASES::real_t> lb_, ub_;
        Eigen::VectorXd last_torque_ = Eigen::VectorXd::Zero(6);
        int N_, d_, n_, m_;
        double dt_;
        bool first_run_ = true;
    };
}
// int main() {
//     // 1. 初始化MPC控制器 (预测步长20, 控制周期0.01s, 延迟3步)
//     UR5VibrationMPC mpc(20, 0.01, 3);

//     // 2. 实时控制循环
//     while (true) {
//         Eigen::VectorXd current_state(18 + 18); // [q, dq, ddq, τ_hist]
//         current_state << get_joint_pos(), get_joint_vel(), get_joint_accel(), 
//                          get_historical_torques();

//         Eigen::VectorXd target = get_target_joint_pos();

//         // 3. 求解最优控制力矩 (振动抑制)
//         Eigen::VectorXd torque = mpc.solve(current_state, target);

//         // 4. 输出到执行器
//         set_motor_torque(torque);

//         // 5. 等待下一个控制周期
//         sleep(0.01); 
//     }
//     return 0;
// }

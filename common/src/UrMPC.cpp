#include "UrMPC.hpp"

UR5VibrationMPC::UR5VibrationMPC(int horizon, double dt, int delay_steps)
    : N_(horizon), dt_(dt), d_(delay_steps)
{
    // 初始化动力学参数 (UR5示例值)
    M_ = Eigen::MatrixXd::Identity(6, 6) * 2.0;
    C_ = Eigen::MatrixXd::Identity(6, 6) * 0.1;
    K_ = Eigen::MatrixXd::Identity(6, 6) * 50.0;
    D_ = Eigen::MatrixXd::Identity(6, 6) * 0.5; // 振动阻尼

    // 构建扩展状态矩阵 [q, dq, ddq, τ_{k-1}, ..., τ_{k-d}]
    buildExtendedModel();
    precomputeQPWeights(10.0, 0.1, 0.01); // Q_pos, R_jerk, S_torque

    // 初始化qpOASES求解器
    solver_ = new qpOASES::SQProblem(N_ * m_, 0); // 无等式约束
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    solver_->setOptions(options);
}

Eigen::VectorXd UR5VibrationMPC::solve(const Eigen::VectorXd &current_state,
                                       const Eigen::VectorXd &target_joint)
{
    // 1. 构建参考轨迹 (N步恒定目标)
    Eigen::VectorXd ref_trajectory = Eigen::VectorXd::Zero(N_ * n_);
    for (int i = 0; i < N_; ++i)
    {
        ref_trajectory.segment(i * n_, 6) = target_joint; // q_ref
    }

    // 2. 计算梯度向量 g = 2 * MᵀQ (S·x_current - ref)
    Eigen::VectorXd g = M_.transpose() * Q_ * (S_ * current_state - ref_trajectory);
    std::vector<qpOASES::real_t> g_qp(g.data(), g.data() + g.size());

    // 3. 热启动求解QP
    int nWSR = 100;
    qpOASES::returnValue status;
    if (first_run_)
    {
        status = solver_->init(H_.data(), g_qp.data(),
                               nullptr, lb_.data(), ub_.data(),
                               nullptr, nullptr, nWSR);
        first_run_ = false;
    }
    else
    {
        status = solver_->hotstart(g_qp.data(), lb_.data(), ub_.data(),
                                   nullptr, nullptr, nWSR);
    }

    // 4. 获取最优控制序列的首元素 (关节力矩)
    Eigen::VectorXd torque_opt = last_torque_;
    if (status == qpOASES::SUCCESSFUL_RETURN)
    {
        std::vector<qpOASES::real_t> u_opt(N_ * m_);
        solver_->getPrimalSolution(u_opt.data());
        torque_opt = Eigen::Map<Eigen::VectorXd>(u_opt.data(), m_);
        last_torque_ = torque_opt;
    }
    return torque_opt;
}

void UR5VibrationMPC::buildExtendedModel()
{
    n_ = 18 + 6 * d_; // q(6) + dq(6) + ddq(6) + d个历史力矩
    m_ = 6;           // 控制输入维度 (关节力矩)

    // 连续系统矩阵 A_c (简化为线性化模型)
    A_c_ = Eigen::MatrixXd::Zero(n_, n_);
    A_c_.block(0, 6, 6, 6) = Eigen::MatrixXd::Identity(6, 6);  // dq = dq
    A_c_.block(6, 12, 6, 6) = Eigen::MatrixXd::Identity(6, 6); // ddq = ddq
    A_c_.block(12, 12, 6, 6) = -M_.inverse() * D_;             // dddq = -M⁻¹D·ddq
    A_c_.block(12, 18, 6, 6) = M_.inverse();                   // dddq += M⁻¹τ

    // 离散化: A_d = I + A_c * dt
    A_d_ = Eigen::MatrixXd::Identity(n_, n_) + A_c_ * dt_;

    // 控制输入矩阵 B_d
    B_d_ = Eigen::MatrixXd::Zero(n_, m_);
    B_d_.block(12, 0, 6, 6) = M_.inverse() * dt_; // τ -> dddq
}

void UR5VibrationMPC::precomputeQPWeights(double Q_pos, double R_jerk, double S_torque)
{
    // 预测矩阵 S 和 M
    S_ = Eigen::MatrixXd::Zero(N_ * 6, n_); // 仅跟踪关节位置
    E_ = Eigen::MatrixXd::Zero(N_ * 6, N_ * m_);
    Eigen::MatrixXd A_power = A_d_;

    for (int i = 0; i < N_; ++i)
    {
        S_.block(i * 6, 0, 6, n_) = A_power.block(0, 0, 6, n_);
        for (int j = 0; j <= i; ++j)
        {
            if (i - j < d_)
                continue; // 忽略延迟步长内的影响
            E_.block(i * 6, j * m_, 6, m_) = A_d_.block(0, 18 + (i - j - 1) * m_, 6, m_) * B_d_;
        }
        A_power = A_d_ * A_power;
    }

    // 代价函数矩阵: H = 2*(MᵀQ M + R)
    Q_ = Eigen::MatrixXd::Identity(N_ * 6, N_ * 6) * Q_pos;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(N_ * m_, N_ * m_) * S_torque;
    H_ = 2 * (E_.transpose() * Q_ * E_ + R);

    // 振动抑制项: 显式惩罚dddq (Jerk)
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(N_ * 6, N_ * 6);
    for (int i = 0; i < N_; ++i)
    {
        J.block(i * 6, i * 6, 6, 6) = E_.inverse() * D_ * R_jerk;
    }
    H_ += 2 * E_.transpose() * J * E_;

    // 控制量边界
    lb_.resize(N_ * m_, -100.0); // 力矩下限
    ub_.resize(N_ * m_, 100.0);  // 力矩上限
}
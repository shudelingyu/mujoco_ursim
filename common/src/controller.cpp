#include "controller.hpp"
#include <algorithm>
#include "algorithm/math_util.hpp"
#include <cstring>
#include <stdexcept>
#include <iostream>

namespace common {

Controller& Controller::getInstance() {
    static Controller instance;
    return instance;
}

Controller::Controller() {
    // 初始化默认参数
    m_ctrl.joints.resize(m_num_joints);
}

void Controller::resize_vectors(size_t size) {
    m_num_joints = size;
    m_ctrl.joints.resize(size);
}

void Controller::reset() {  
    for (auto& joint : m_ctrl.joints) {
        joint.position_pid.prev_error = 0.0;
        joint.position_pid.integral = 0.0;
        joint.velocity_pid.prev_error = 0.0;
        joint.velocity_pid.integral = 0.0;
    }
}

void Controller::load_xml(const InitConfig &init, MujocoContext &context, int argc, const char** argv) {
    std::string xmlpath = init.xml_file;
    char error[1000] = "Could not load model";

    if (argc < 2) {
        context.model = mj_loadXML(xmlpath.c_str(), nullptr, error, sizeof(error));
    } 
    else if (std::strlen(argv[1]) > 4 && !strcmp(argv[1] + strlen(argv[1]) - 4, ".mjb")) {
        context.model = mj_loadModel(argv[1], nullptr);
    } 
    else {
        context.model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
    }

    if (!context.model) {
        throw std::runtime_error("Load model error: " + std::string(error));
    }

    context.data = mj_makeData(context.model);
    resize_vectors(context.model->nu);  // 自动适配关节数
}

void Controller::configure_joint_pid(size_t joint_index, double Kpp, double Kpd, double Kpi, double Kvp, double Kvd) {
    if (joint_index >= m_num_joints) {
        throw std::out_of_range("Joint index out of bounds");
    }

    auto& joint = m_ctrl.joints[joint_index];
    switch (joint.mode) {
        case ControlMode::POSITION:
            joint.position_pid.Kp = Kpp;
            joint.position_pid.Kd = Kpd;
            joint.position_pid.Ki = Kpi;
            break;
            
        case ControlMode::VELOCITY:
            joint.velocity_pid.Kp = Kvp;
            joint.velocity_pid.Kd = Kvd;
            break;
            
        case ControlMode::TORQUE:
            // 扭矩模式无需PID参数
            break;
    }
}

void Controller::configure_joints_mode(ControlMode mode) {
    for (auto& joint : m_ctrl.joints) {
        joint.mode = mode;
    }
}

void Controller::configure_joint_vel_limit(size_t joint_index, const std::string& vel_type, double vel_limit) {
    if (joint_index >= m_num_joints) {
        throw std::out_of_range("Joint index out of bounds");
    }
    
    auto& joint = m_ctrl.joints[joint_index];
    
    if (vel_type == "RPM") {
        joint.velocity_limit = vel_limit / 60.0 * 2.0 * algorithms::PI;
    } 
    else if (vel_type == "degree") {
        joint.velocity_limit = vel_limit * algorithms::DEG_TO_RAD;
    }
    else {
        joint.velocity_limit = vel_limit;
    }
}

void Controller::set_joint_targets(
    size_t joint_index,
    std::optional<double> position,
    std::optional<double> velocity,
    std::optional<double> torque
) {
    if (joint_index >= m_num_joints) {
        throw std::out_of_range("Joint index out of bounds");
    }

    auto& joint = m_ctrl.joints[joint_index];
    
    // 更新目标值（仅当参数非空时更新）
    if (position.has_value()) {
        joint.target_position = position.value();
    }
    if (velocity.has_value()) {
        joint.target_velocity = velocity.value();
    }
    if (torque.has_value()) {
        joint.target_torque = torque.value();
    }
}

// ================ 核心控制计算接口 ================

void Controller::compute_control(const mjModel* model, mjData* data) {
    const double dt = model->opt.timestep;
    
    for (size_t i = 0; i < m_num_joints; i++) {
        auto& joint = m_ctrl.joints[i];
        
        // 更新当前状态
        joint.current_position = data->qpos[i];
        joint.current_velocity = data->qvel[i];
        
        // 根据控制模式计算控制信号
        switch (joint.mode) {
            case ControlMode::TORQUE:
                data->ctrl[i] = compute_torque_control(joint);
                break;
                
            case ControlMode::VELOCITY:
                data->ctrl[i] = compute_velocity_control(joint, data->qvel[i], dt);
                break;
                
            case ControlMode::POSITION:
                data->ctrl[i] = compute_position_control(joint, data->qpos[i], data->qvel[i], 
                                                          data->qfrc_bias[i], dt);
                break;
        }
    }
}

// ================ 具体控制算法实现 ================

double Controller::compute_torque_control(const JointController& joint) {
    // 直接返回目标扭矩
    return joint.target_torque;
}

double Controller::compute_velocity_control(const JointController& joint, double current_vel, double dt) {
    // 速度控制模式
    double velocity_error = joint.target_velocity - current_vel;
    double derivative = (velocity_error - joint.velocity_pid.prev_error) / dt;
    
    // 更新PID状态（注意：这里修改了const引用，实际应避免）
    // 更好的设计是返回PID状态更新，但为简化保持这样
    const_cast<JointController&>(joint).velocity_pid.prev_error = velocity_error;
    
    return joint.velocity_pid.Kp * velocity_error + 
           joint.velocity_pid.Kd * derivative;
}

double Controller::compute_position_control(const JointController& joint, 
                                            double current_pos, double current_vel, 
                                            double bias, double dt) {
    // 位置控制模式
    double position_error = joint.target_position - current_pos;
    double velocity_error = joint.target_velocity - current_vel;
    
    // 比例项
    double proportional = joint.position_pid.Kp * position_error;
    
    // 微分项
    double derivative = (position_error - joint.position_pid.prev_error) / dt;
    
    // 更新PID状态
    const_cast<JointController&>(joint).position_pid.prev_error = position_error;
    
    // PID输出
    double pid_output = proportional + joint.position_pid.Kd * derivative;
    
    // 重力补偿
    double gravity_compensation = bias;
    
    // 最终控制输出
    return pid_output + gravity_compensation;
}

} // namespace common
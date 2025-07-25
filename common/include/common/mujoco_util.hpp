#pragma once
#include <GLFW/glfw3.h>
#include <algorithm>
#include <cstring>
#include <iostream>
#include <map>
#include <mujoco/mujoco.h>
#include <vector>

namespace common {

// 控制模式枚举
enum class ControlMode {
    TORQUE,
    VELOCITY,
    POSITION
};

// 初始化配置结构体
struct InitConfig {
    std::string xml_file = "./model/ur/ur5.xml";
    std::string data_file = "data.csv";
    double sim_duration = 20.0;
    double radius = 0.5;
    double angular_velocity = 0.5;
    
    // 数据记录相关
    FILE* data_stream = nullptr;
    int loop_count = 0;
    int data_log_freq = 10;
};

// Mujoco 封装结构体
struct MujocoContext {
    mjModel* model = nullptr;
    mjData* data = nullptr;
    mjvCamera camera;
    mjvOption options;
    mjvScene scene;
    mjrContext render_context;
    
    // 控制相关变量
    float control_update_freq = 100.0f;
    double last_update_time = 0.0;
    double control_value;
    mjvPerturb perturbation;
};

// PID 参数结构体
struct PIDParams {
    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;
    double prev_error = 0.0;
    double integral = 0.0;
    double max_integral = 100.0;
    double max_output = 30.0;
    double target = 0.0;
    double current = 0.0;
};

// 关节控制器结构体
struct JointController {
    ControlMode mode = ControlMode::POSITION;
    double target_position = 0.0;    // rad
    double target_velocity = 0.0;    // rad/s
    double target_torque = 0.0;       // Nm
    double velocity_limit = 0.33;     // rad/s
    double current_position = 0.0;
    double current_velocity = 0.0;
    
    PIDParams position_pid;  // 位置环PID
    PIDParams velocity_pid;   // 速度环PID
};

struct RobotData
{
    std::vector<int> body_parentid_;

    std::vector<int> jnt_type_;
    std::vector<int> jnt_group_;
    std::vector<int> jnt_qposadr_;
    std::vector<std::optional<std::pair<mjtNum, mjtNum>>> jnt_range_;
    std::vector<std::string> jnt_names_;

    std::vector<int> actuator_group_;
    std::vector<std::optional<std::pair<mjtNum, mjtNum>>> actuator_ctrlrange_;
    std::vector<std::string> actuator_names_;

    std::vector<std::string> equality_names_;

    std::vector<mjtNum> history_; // history buffer (nhistory x state_size)

    // mjModel and mjData fields that can be modified by the user through the GUI
    std::vector<mjtNum> qpos_;
    std::vector<mjtNum> qpos_prev_;
    std::vector<mjtNum> ctrl_;
    std::vector<mjtNum> ctrl_prev_;
    std::vector<mjtByte> eq_active_;
    std::vector<mjtByte> eq_active_prev_;
};

// 机器人控制状态
struct RobotControlState {
    std::vector<JointController> joints; // 关节控制器向量
    int active_joint_index = 0;          // 当前激活的关节索引
    RobotData robot_data_;
};

} // namespace mujoco_utils
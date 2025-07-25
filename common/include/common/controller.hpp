#pragma once
#include "mujoco_util.hpp"
#include <vector>
#include <string>

namespace common {

class Controller
{
public:
    static Controller& getInstance();

    void reset();
    void load_xml(const InitConfig &init);
    void init(mjModel* model, mjData* data);
    void configure_joint_pid(size_t joint_index, double Kpp, double Kpd, double Kpi, double Kvp, double Kvd);
    void configure_joints_mode(ControlMode mode);
    void configure_joint_vel_limit(size_t joint_index, const std::string& vel_type, double vel_limit);
    void set_joint_targets(
        size_t joint_index,
        std::optional<double> position = std::nullopt,
        std::optional<double> velocity = std::nullopt,
        std::optional<double> torque = std::nullopt
    );
    // 新增：统一控制计算接口
    void compute_control(const mjModel* model, mjData* data);

    InitConfig get_config(){return init_config_;}
    RobotControlState* get_control() { return &rst_; }
    MujocoContext *get_mujocoContext(){return &mujoco_context_;}

private:
    Controller();
    void resize_vectors(size_t size);
    // 控制计算方法（按模式）
    double compute_torque_control(const JointController& joint);
    double compute_velocity_control(const JointController& joint, double current_vel, double dt);
    double compute_position_control(const JointController& joint, double current_pos, double current_vel, double bias, double dt);
private:
    mjModel* m_ = nullptr;
    mjData* d_ = nullptr;
    size_t joints_num_ = 0;
    RobotControlState rst_;
    MujocoContext mujoco_context_;
    InitConfig init_config_;
};

} // namespace common
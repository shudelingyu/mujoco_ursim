#pragma once
#include "math_util.hpp"

class Controller
{
    public:
    Controller();
    ~Controller();
    public:
    void load_xml(const initmujoco &init,MjWrapper &mjWrapper,int argc, const char** argv);
    void configure_actuators_pid(const mjModel *m, int selected_joint, double Kpp, double Kpd, double Kvp, double Kvd);
    void configure_actuators_mode(const mjModel* m, ControlMode mode);
    void configure_actuators_vel_limit(const mjModel* m, const std::string& vel_type,int selected_joint, double vel_limit);
    double PDcontroller(JointController& ctrl,const double& dt);
    double PIcontroller(JointController& ctrl,const double& dt);
    g_control* get_ctrl(){return &m_ctrl;}

    private:
    g_control m_ctrl;
};


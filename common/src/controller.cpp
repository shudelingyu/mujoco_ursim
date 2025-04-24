#include "controller.hpp"

Controller::Controller()
{
    m_ctrl.joints.resize(6);
}

Controller::~Controller()
{
}

void Controller::load_xml(const initmujoco &init,MjWrapper &mjWrapper,int argc, const char** argv){
    char xmlpath[100] = {};
    char datapath[100] = {};

    strcat(xmlpath, init.path.c_str());
    strcat(xmlpath, init.xmlfile.c_str());

    strcat(datapath, init.path.c_str());
    strcat(datapath, init.datafile.c_str());
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if (argc < 2)
        mjWrapper.model = mj_loadXML(xmlpath, 0, error, 1000);

    else if (strlen(argv[1]) > 4 && !strcmp(argv[1] + strlen(argv[1]) - 4, ".mjb"))
        mjWrapper.model  = mj_loadModel(argv[1], 0);
    else
        mjWrapper.model  = mj_loadXML(argv[1], 0, error, 1000);
    if (!mjWrapper.model )
        mju_error_s("Load model error: %s", error);

    // make data
    mjWrapper.data = mj_makeData(mjWrapper.model);
}

// 配置制定关节pd参数
void Controller::configure_actuators_pid(const mjModel *m, int selected_joint, double Kpp, double Kpd, double Kvp, double Kvd)
{
    auto &ctrl = m_ctrl.joints[selected_joint];
    switch (ctrl.mode)
    {
    case TORQUE_CONTROL:
        break;
    case VELOCITY_CONTROL:
    {
        ctrl.pid.Kvp = Kvp;
        ctrl.pid.Kvd = Kvd;
        break;
    }

    case POSITION_CONTROL:
    {
        ctrl.pid.Kpp = Kpp;
        ctrl.pid.Kpd = Kpd;
        break;
    }
    }
}

// 配置执行器控制模式
void Controller::configure_actuators_mode(const mjModel *m, ControlMode mode)
{
    for (int i = 0; i < m->nu; i++)
    {
        switch (mode)
        {
        case TORQUE_CONTROL:
            m_ctrl.joints[i].mode = TORQUE_CONTROL;
            break;
        case VELOCITY_CONTROL:
            m_ctrl.joints[i].mode = VELOCITY_CONTROL;
            break;
        case POSITION_CONTROL:
            m_ctrl.joints[i].mode = POSITION_CONTROL;
            break;
        }
    }
}

void Controller::configure_actuators_vel_limit(const mjModel *m, const std::string &vel_type, int selected_joint, double vel_limit)
{
    auto &joint = m_ctrl.joints[selected_joint];
    if (vel_type == "RPM")
    {
        joint.vel_limit = vel_limit / 60 * 2 * M_PI; // RPM->rad
    }
    else if (vel_type == "rad")
    {
        joint.vel_limit = vel_limit; // rad->rad
    }
    else if (vel_type == "degree")
    {
        joint.vel_limit = vel_limit / 360 * 2 * M_PI; // degree->rad
    }
}

double Controller::PDcontroller(JointController &ctrl, const double &dt)
{
    double pos_error = ctrl.target_pos - ctrl.current_pos;
    double derivative = (pos_error - ctrl.pid.prev_error) / dt;
    ctrl.pid.prev_error = pos_error;
    return ctrl.pid.Kpp * pos_error + ctrl.pid.Kpd * derivative;
}

double Controller::PIcontroller(JointController &ctrl, const double &dt)
{
    double vel_error = ctrl.target_vel - ctrl.current_vel;
    ctrl.pid.integral += vel_error * dt;
    ctrl.pid.integral = std::clamp(ctrl.pid.integral, -ctrl.pid.max_integral, ctrl.pid.max_integral);
    double output = ctrl.pid.Kpp * vel_error + ctrl.pid.Kpi * ctrl.pid.integral;
    // 输出限幅
    return std::clamp(output, -ctrl.pid.max_output, ctrl.pid.max_output);
}
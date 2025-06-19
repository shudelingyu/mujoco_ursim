#pragma once
#include "math_util.hpp"

class Controller
{
    public:
    static Controller& getInstance() {
        static Controller instance; // C++11 保证静态局部变量线程安全
        return instance;
    }

    // 删除拷贝构造函数和赋值运算符，防止复制单例对象
    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;

    void reset();
    void load_xml(const initmujoco &init,MjWrapper &mjWrapper,int argc, const char** argv);
    void configure_actuators_pid(const mjModel *m, int selected_joint, double Kpp, double Kpd, double Kpi,double Kvp, double Kvd);
    void configure_actuators_mode(const mjModel* m, ControlMode mode);
    void configure_actuators_vel_limit(const mjModel* m, const std::string& vel_type,int selected_joint, double vel_limit);
    double PDcontroller(JointController& ctrl,const double& dt);
    double PIcontroller(JointController& ctrl,const double& dt);
    void AdmittanceController(const mjModel* m,mjData* d,const double& dt,AdmittanceParam& param,std::vector<double> &target_q);
    g_control* get_ctrl(){return &m_ctrl;}

    private:
    Controller();
    ~Controller();

    private:
    g_control m_ctrl;
    std::vector<double> prev_ddxe;
    std::vector<double> prev_dxe;
    std::vector<double> prev_xe;
    
};


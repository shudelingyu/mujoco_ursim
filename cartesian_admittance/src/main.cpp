
// //c++版本进行测试PID控制器
#include <GLFW/glfw3.h>
#include "math_util.hpp"
#include "plan.hpp"
#include "ur_kinematics.hpp"
#include "controller.hpp"
#include "mujoco_view.hpp"

using namespace mr;
Controller& m_controller = Controller::getInstance();
Planning plan;
initmujoco initmu;
MjWrapper _mujoco;
AdmittanceParam Param;


//Change the path <template_writeData>
//Change the xml file
 //double qinit[6] ={0, -2.55, -0.785, 0, 0, 0};
//double qinit[6] = {0,-1,0,1,0,0};
// const double qinit[6] = {0,0,0,0,0,0};
// double qinit[6] ={-0.785, -2.55, -0.785, -4.71, -1.57, 0};
// double qinit[6] ={0, -1, 1.57, 0, -1.57, 0};

// double qinit[6] = {M_PI*0.5,M_PI*0.5,M_PI*0.5,M_PI*0.5,M_PI*0.5,0};
// double qinit[6] = {-M_PI*0.5,-M_PI*0.5,0,-M_PI*0.5,0,0};
double qinit[6] = {0,-M_PI*0.4,M_PI*0.4,M_PI*0.45,M_PI*0.5,0};
// double qinit[6] ={-0.785, -2.55, -0.785, -4.71, -1.57, 0};
//frequency at which data is written to a file



// 控制器回调
void controller(const mjModel* m, mjData* d) {
    double time = d->time;
    double dt = 0.002;
    std::vector<double> target_q;
    m_controller.AdmittanceController(m,d,dt,Param,target_q);
 
    std::cout<<"target_q";
    for(auto &i : target_q)
        std::cout<<"  "<<i;
    std::cout<<std::endl;

    for (int i = 0; i < m->nu; i++) {
        auto& ctrl = m_controller.get_ctrl()->joints[i];

        ctrl.target_pos= target_q[i];
        ctrl.pid.target = ctrl.target_pos;
        ctrl.pid.current = d->qpos[i];
        switch (ctrl.mode) {
            case TORQUE_CONTROL:
                d->ctrl[i] = ctrl.target_torque;
                std::cout<<time<<"  "<<d->ctrl[i]<<"  "<<d->qpos[i]<<"  "<<d->actuator_force[i]<<std::endl;
                break;
            case VELOCITY_CONTROL: {
                double error = ctrl.target_vel - d->qvel[i];
                double derivative = (error - ctrl.pid.prev_error) / m->opt.timestep;
                ctrl.pid.prev_error = error;
                d->ctrl[i] = ctrl.pid.Kvp * error + ctrl.pid.Kvd * derivative;
                break;
                }
            case POSITION_CONTROL: {
                d->ctrl[i]=m_controller.PDcontroller(ctrl,dt)+d->qfrc_bias[i];
                break;
            }
        }
    }

}

void init_controller(const mjModel* m, mjData* d)
{
    mj_forward(m, d);
    UrParam ur_dh;
    ur_dh.L1 = 0.425;
    ur_dh.L2 = 0.39225;
    ur_dh.W1 = 0.10915;
    ur_dh.W2 = 0.0823;
    ur_dh.H1 = 0.089159;
    ur_dh.H2 = 0.09465;
    ur_frame::init(ur_dh);
    m_controller.configure_actuators_mode(m, ControlMode::POSITION_CONTROL);
    double joint_Kpp[] = {200, 200, 200, 200, 200, 200};
    double joint_Kpd[] = {100, 100, 100, 100, 100, 100};
    double joint_Kpi[] = {10, 10, 10, 10, 10, 10};

    double joint_Kvp[] = {200, 200, 200, 200, 200, 20};
    double joint_Kvd[] = {10, 10, 10, 10, 10, 10};
    double joint_vellimit[] = {120, 120, 120, 120, 120, 120};

    Param.Fe = {0,0,5,0,0,0};
    Param.K = {10, 30, 30, 10, 10, 10};
    Param.B = {10, 10, 5, 10, 10, 10};


    for (int i = 0; i < m->nu; i++)
    {
        m_controller.configure_actuators_pid(m, i, joint_Kpp[i], joint_Kpd[i], joint_Kpi[i], joint_Kvp[i], joint_Kvd[i]);
        m_controller.configure_actuators_vel_limit(m, "degree", i, joint_vellimit[i]);
    }
}

//************************
// main function
int main(int argc, const char** argv)
{
    initmu.simend = 50;
    m_controller.load_xml(initmu,_mujoco,argc,argv);

    for (size_t i = 0; i < 6; i++)
    {
        _mujoco.data->qpos[i] = qinit[i];
        _mujoco.data->qvel[i]=0;
        _mujoco.data->qacc[i]=0;
    }
    MuJoCoViewer view(_mujoco);

    // 初始化控制器
    init_controller(_mujoco.model,_mujoco.data);
    // 设置控制器
    mjcb_control = controller;

    view.run(initmu.simend);

    // free visualization storage
    mjv_freeScene(&_mujoco.scn);
    mjr_freeContext(&_mujoco.con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(_mujoco.data);
    mj_deleteModel(_mujoco.model);


    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}

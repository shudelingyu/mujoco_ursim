#include "controller.hpp"
#include "ur_kinematics.hpp"
#include "modern_robotics.hpp"
// #include "ACSP.hpp"

Controller::Controller()
{
    m_ctrl.joints.resize(6);
    prev_ddxe.resize(6);
    prev_ddxe.reserve(0);
    prev_dxe.resize(6);
    prev_dxe.reserve(0);
    prev_xe.resize(6);
    prev_xe.reserve(0);
}

Controller::~Controller()
{
}

void Controller::reset(){
    prev_ddxe.resize(6);
    prev_ddxe.reserve(0);
    prev_dxe.resize(6);
    prev_dxe.reserve(0);
    prev_xe.resize(6);
    prev_xe.reserve(0);
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
void Controller::configure_actuators_pid(const mjModel *m, int selected_joint, double Kpp, double Kpd,double Kpi, double Kvp, double Kvd)
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
        ctrl.pid.Kpi = Kpi;

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
    double pos_error = ctrl.pid.target - ctrl.pid.current;
    double derivative = (pos_error - ctrl.pid.prev_error) / dt;
    ctrl.pid.prev_error = pos_error;
    return ctrl.pid.Kpp * pos_error + ctrl.pid.Kpd * derivative;
}

double Controller::PIcontroller(JointController &ctrl, const double &dt)
{
    double vel_error = ctrl.pid.target - ctrl.pid.current;
    ctrl.pid.integral += vel_error * dt;
    ctrl.pid.integral = std::clamp(ctrl.pid.integral, -ctrl.pid.max_integral, ctrl.pid.max_integral);
    double output = ctrl.pid.Kpp * vel_error + ctrl.pid.Kpi * ctrl.pid.integral;
    // 输出限幅
    return std::clamp(output, -ctrl.pid.max_output, ctrl.pid.max_output);
}

void Controller::AdmittanceController(const mjModel* m,mjData* d,const double& dt,AdmittanceParam& param,std::vector<double> &target_q){
    // ddxe(i+1)=[Fxe(i+1)-B*dxe(i)-K*xe(i)]/M;
    // dxe(i+1)=dt*[ddxe(i+1)+ddxe(i)]/2+dxe(i);              %v1=dt*(a0+a1)/2+v0  加速度一次积分得速度
    // xe(i+1)=dt*[dxe(i+1)+dxe(i)]/2+xe(i);
    target_q.clear();
    std::cout<<"param.dxe";
    for(int i = 0 ;i<6;i++){
        // prev_ddxe[i] = param.ddxe[i];
        // prev_dxe[i] = param.dxe[i];
        // prev_xe[i] = param.xe[i];
        // param.ddxe[i] = (param.Fe[i]-param.B[i]*param.dxe[i]-param.K[i]*param.xe[i])/param.M[i];
        // param.dxe[i] = dt*(param.ddxe[i]+prev_ddxe[i])/2 + prev_dxe[i];
        // param.xe[i] = dt*(param.dxe[i]+prev_dxe[i])/2+prev_xe[i];
        // std::cout<<" "<<param.dxe[i];
        param.dxe[i] = param.Fe[i]/param.K[i];
        param.xe[i] = dt*param.dxe[i];
        std::cout<<" "<<param.dxe[i];
    }
    std::cout<<std::endl;
    //利用雅可比矩阵求关节速度积分到关节位置
//     Eigen::MatrixXd J = ur_frame::instance()->jacobian_body(m,d);
//     vec_t qv = pinv(J)*stdvec2vecxd(param.dxe);
//    // 限制关节速度
//     double max_velocity = 1.0; // 假设最大速度为 1 rad/s
//     for (int i = 0; i < qv.size(); i++) {
//         qv[i] = std::clamp(qv[i], -max_velocity, max_velocity);
//     }
//     std::cout<<"qv"<<qv.transpose().matrix()<<std::endl;
//     // theta_offset=theta_offset+qv*dt;
//     vec_t target(6);
//     for(int i=0;i<6;i++){
//         target[i] = d->qpos[i] + qv[i]*dt;
//     }
//     target_q = vecxd2stdvec(target);

    // 利用逆解求关节位置
    // param.xe = {0,0,0.0000000000001,0,0,0};
    tf_t error_ee = PosAxisAngToTrans(stdvec2vecxd(param.xe));
    
    std::cout<<"error_ee===="<<std::endl<<error_ee.matrix()<<std::endl;
    tf_t cur_ee = ur_frame::instance()->forward_kinematics(m,d);
    std::cout<<"cur_ee===="<<std::endl<<cur_ee.matrix()<<std::endl;
    tf_t target_ee = cur_ee * error_ee;
    std::cout<<"target_ee===="<<std::endl<<target_ee.matrix()<<std::endl;
    double target_ee_d[16]{0};
    MatrixXdToDoubleArray(target_ee,target_ee_d);

    // target_q = vecxd2stdvec(ur_frame::instance()->select_sln(target_ee_d,d));
    ur_frame::instance()->select_sln(target_ee,d,target_q);

}
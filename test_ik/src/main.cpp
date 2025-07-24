
// //c++版本进行测试PID控制器
#include <GLFW/glfw3.h>
#include "math_util.hpp"
#include "plan.hpp"
#include "ur_kinematics.hpp"
#include "controller.hpp"
#include "mujoco_view.hpp"
#include "inputshaper.hpp"

using namespace mr;
Controller& m_controller = Controller::getInstance();
Planning plan;
initmujoco initmu;
ServoSeriesPolynomial3Param param;
MjWrapper _mujoco;
InputShaper inputshaper;



//Change the path <template_writeData>
//Change the xml file
 //double qinit[6] ={0, -2.55, -0.785, 0, 0, 0};
//double qinit[6] = {0,-1,0,1,0,0};
const double qinit[6] = {0,0,0,0,0,0};
// double qinit[6] ={-0.785, -2.55, -0.785, -4.71, -1.57, 0};

// double qinit[6] = {M_PI*0.5,M_PI*0.5,M_PI*0.5,M_PI*0.5,M_PI*0.5,0};
//double qinit[6] = {-M_PI*0.5,-M_PI*0.5,0,-M_PI*0.5,0,0};
//double qinit[6] = {0,-M_PI*0.5,M_PI*0.5,0,M_PI*0.5,0};
// double qinit[6] ={-0.785, -2.55, -0.785, -4.71, -1.57, 0};
//frequency at which data is written to a file




// 控制器回调
void controller(const mjModel* m, mjData* d) {
    double time = d->time;
    double dt = 0.002;
    // ur_frame *ur_kin = ur_frame::instance();
    //正动力学计算
    //mj_forward(m,d);
    //FK
    // Eigen::MatrixXd cur_ee=ur_kin->forward_kinematics(m,d);
    // std::cout<<"cur_ee"<<std::endl<<cur_ee.matrix()<<std::endl;
    // double ee_pm[16]{ 0 };
    // MatrixXdToDoubleArray(cur_ee,ee_pm); 
    // for (int i = 0; i < 16; ++i) {
    //     std::cout << ee_pm[i] << " ";
    //     if ((i + 1) % 4 == 0) std::cout << std::endl;
    // }
    // //Ik
    // Eigen::VectorXd thetaList=ur_kin->select_sln(ee_pm,d);
    // std::cout<<"tar_q"<<thetaList.transpose().matrix()<<std::endl;
    // std::vector<double> sln;
    // if(!ur_kin->select_sln(cur_ee,d,sln))
    //     std::cout<<"no inverse"<<std::endl;
    // else
    //     std::cout<<"tar_q2"<<stdvec2vecxd(sln).transpose().matrix()<<std::endl;

    // std::cout<<"time=========="<<time <<std::endl;

    // 逆动力学求解
    //mj_inverse(m,d);
    // double dense_M[m->nv*m->nv] = {0};
    // mj_fullM(m,dense_M, d->qM);

    double target_pos[6] = {0, 0, 0, 0, 0, 0}; 
    double target_vel[6] = {0, 0, 0, 0, 0, 0}; 
    double target_acc[6] = {0, 0, 0, 0, 0, 0}; 
 
    for (int i = 0; i < m->nu; i++) {
        auto& ctrl = m_controller.get_ctrl()->joints[i];

        ctrl.target_pos= plan.CubicSpline_at(param.time_whole.size(), param.time_whole.data(), param.pos[i].data(), param.vel[i].data(), param.pos_p1[i].data(), param.pos_p2[i].data(), param.pos_p3[i].data(), time);
        if(i==1){
            ctrl.target_pos = inputshaper.process(ctrl.target_pos);
            std::cout<<"time=========="<<time <<std::endl;
        }
        // ctrl.target_pos = inputshaper.process(ctrl.target_pos);
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
                // ctrl.target_pos=0 + 0.5*sin(2*M_PI*0.5*time);/* code */
                // ctrl.target_vel=0 + 2*M_PI*0.5*0.5*cos(2*M_PI*0.5*time);/* code */
                // double pos_error = ctrl.target_pos - d->qpos[i];
                // double pos_control=ctrl.pid.Kpp * pos_error;
                // //pos_control=std::clamp(pos_control,-ctrl.vel_limit,ctrl.vel_limit);
                // //tor_limit
                // double vel_error = ctrl.target_vel - d->qvel[i];
                // double vel_control=ctrl.pid.Kpd* vel_error;
                //pd控制
                // d->qpos[i] = qinit[i];
                // d->ctrl[i] = pos_control + vel_control+d->qfrc_bias[i];
                // ctrl.pid.target = m_controller.PIcontroller(ctrl,dt);
                // ctrl.pid.current = d->qvel[i];
                //pd控制+Feedforward
                //vel_control=std::clamp(vel_control,m->actuator_ctrlrange[2 * i],m->actuator_ctrlrange[2 * i + 1]);
                // target_pos[i]=ctrl.target_pos;
                // target_vel[i]=0;
                // target_acc[i]=0;
                d->ctrl[i]=m_controller.PDcontroller(ctrl,dt)+d->qfrc_bias[i];
                // d->ctrl[i] = pos_control + vel_control+d->qfrc_bias[i];

                //PD + Feedback Linearization
                // std::vector<double> u(m->nv);model-
                // u[i]= pos_control + vel_control+d->qfrc_bias[i];
                // d->ctrl[i] = 0.0;

                // for (int j = 0; j < m->nv; j++) {
                //     d->ctrl[i] += d->qM[m->nv*i + j] * u[j]; // M(q)*u
                // }
                // d->ctrl[i] += d->qfrc_bias[i];
                
                //逆动力学控制
                // M*qacc + qfrc_bias = qfrc_applied + ctrl
                // double vel_freedforward=pos_error;
                // 3. 计算逆动力学
                //std::cout<<" "<<time<<" "<<ctrl.target_pos<<"  "<<d->qpos[i]<<" pos_error "<<(ctrl.target_pos-d->qpos[i])*360/M_PI<<"  pos_control "<<pos_control<<" vel_error "<<vel_error<<" vel_control "<<vel_control<<"  fred_force "<<d->qfrc_bias[i]<<"  force  "<<d->actuator_force[i]<<std::endl;
                // M：广义惯性矩阵（包含质量、转动惯量等）
                // qacc：广义加速度
                // qfrc_bias：科里奥利力 + 重力 + 离心力（C(q, qvel) + G(q)）
                // qfrc_applied：外部施加的广义力
                // ctrl：执行器产生的控制力
                // 偏差力	qfrc_bias	科里奥利力+离心力+重力	C(q, qvel) + G(q)
                // 执行器力	qfrc_actuator	执行器（电机、位置伺服等）产生的力	ctrl * gain
                // 被动力	qfrc_passive	被动元件产生的力（阻尼、摩擦、弹簧）	damping*qvel + stiffness*(q0-q)
                // 外部施加力	qfrc_applied	用户通过代码施加的广义力（如 mjv_applyForce）	用户定义
                // 约束力	qfrc_constraint	约束条件产生的力（接触力、关节限位等）	通过 LCP 求解获得
                // 逆动力学力	qfrc_inverse	逆动力学计算的关节力矩（需调用 mj_inverse）	M*qacc + qfrc_bias
                // std::cout<<"  "<<d->qvel[i]*360/M_PI<<"  "<<(ctrl.target_pos-d->qpos[i])*360/M_PI<<"  bias "<<d->qfrc_bias[i]<<" actuator "<<d->qfrc_actuator[i]<<" passive "<<d->qfrc_passive[i]<<" applied "<<d->qfrc_applied[i]<<" constraint "<<d->qfrc_constraint[i]<<" inverse "<<d->qfrc_inverse[i]<<" "<<d->actuator_force[i]<<std::endl;
                break;
            }
        }
    }
    // std::cout<<std::endl;

    // 设置模型的位置、速度、加速度，进行逆动力学计算
    // mju_copy(d->qpos, target_pos, m->nq);
    // mju_copy(d->qvel, target_vel, m->nv);
    // mju_copy(d->qacc, target_acc, m->nv);

    // // 逆动力学控制，计算关节力矩
    // mj_inverse(m,d);
    // for (size_t i = 0; i < 6; i++)
    // {
    //     //d->ctrl[i] = d->ctrl[i]+d->qfrc_inverse[i];
    //     d->ctrl[i] = d->ctrl[i]+d->qfrc_bias[i];
    //     //d->ctrl[i] = d->ctrl[i];
    //     //d->ctrl[i] = d->qfrc_bias[i];
    // }
}

void init_controller(const mjModel* m, mjData* d)
{
  mj_forward(m,d);
  UrParam ur_dh;
  ur_dh.L1 = 0.425;  
  ur_dh.L2 = 0.39225;
  ur_dh.W1 = 0.10915; 
  ur_dh.W2 = 0.0823;  
  ur_dh.H1 = 0.089159;
  ur_dh.H2 = 0.09465;
  ur_frame::init(ur_dh); 
  m_controller.configure_actuators_mode(m, ControlMode::POSITION_CONTROL);
  double joint_Kpp[]={200,200,200,200,200,200};
  double joint_Kpd[]={100,100,100,100,100,100};
  double joint_Kpi[]={10,10,10,10,10,10};


  double joint_Kvp[]={200,200,200,200,200,20};
  double joint_Kvd[]={10,10,10,10,10,10};
  double joint_vellimit[]={120,120,120,120,120,120};

    //三次多项式插值轨迹规划
    // param.time = {0, 1, 2, 3,4};
    double target_pos[5]= {0, 1, 0, -1, 0};
    double target_vel[5]= {0, 0, 0, 0, 0};
    double time[5]={0,1,2,3,4};
    int waypoit_size=sizeof(target_pos) / sizeof(double);
    int num = m->nu;
    param.joint_set.resize(num);
    param.vel_set.resize(num);
    param.scale = 1;
    param.joint_mat.resize(param.joint_set.size() + 1);
    param.vel_mat.resize(param.vel_set.size() + 1);
    param.vel_mat.resize(num);
    param.pos.resize(num);
    param.vel.resize(num);
    param.pos_p1.resize(num);
    param.pos_p2.resize(num);
    param.pos_p3.resize(num);
    param.time.resize(waypoit_size);


    for (int i = 0; i < m->nu; i++)
    {
        param.joint_mat[i].resize(num,0);
        param.vel_mat[i].resize(num, 0);
    }

    for (int i = 0; i < m->nu; i++)
    {
        //param.joint_mat[i][0]=d->qpos[i];
        //std::cout<<param.joint_set.size() + 1<<" "<< param.joint_mat[i][0];
        for (size_t j = 0; j < waypoit_size; j++)
        {
            param.joint_mat[j][i]=target_pos[j];
            param.vel_mat[j][i]=target_vel[j];
            // std::cout<<" "<< target_pos[j];
        }
        // std::cout<<std::endl;
        //std::cout<<"  "<<param.joint_mat.size()<<std::endl;
    }
    //TargetPos
    for (int i = 0; i < num; i++) {
        param.pos[i].resize(param.joint_mat.size() + 6);
        for (int j = 0; j < waypoit_size; j++)
            param.pos[i][j + 3] = param.joint_mat[j][i];
        param.pos[i][0] = param.pos[i][3];
        param.pos[i][1] = param.pos[i][3];
        param.pos[i][2] = param.pos[i][3];
        *(param.pos[i].end() - 1) = *(param.pos[i].end() - 4);
        *(param.pos[i].end() - 2) = *(param.pos[i].end() - 4);
        *(param.pos[i].end() - 3) = *(param.pos[i].end() - 4);
    }
    //TargetVel
    for (int i = 0; i < num; i++) {
        param.vel[i].resize(param.vel_mat.size() + 6);
        for (int j = 0; j < waypoit_size; j++)
            param.vel[i][j + 3] = param.vel_mat[j][i];
        param.vel[i][0] = param.vel[i][3];
        param.vel[i][1] = param.vel[i][3];
        param.vel[i][2] = param.vel[i][3];
        *(param.vel[i].end() - 1) = *(param.vel[i].end() - 4);
        *(param.vel[i].end() - 2) = *(param.vel[i].end() - 4);
        *(param.vel[i].end() - 3) = *(param.vel[i].end() - 4);
    }

    param.time_whole.resize(param.time.size() + 6);
    param.time_whole[0] = -3;
    param.time_whole[1] = -2;
    param.time_whole[2] = -1;
    param.time_whole[3] = time[0];

    for (int i = 1; i < param.time.size(); ++i)
        param.time_whole[i + 3] = time[i] * param.scale;


    *(param.time_whole.end() - 3) = *(param.time_whole.end() - 4) + 1;
    *(param.time_whole.end() - 2) = *(param.time_whole.end() - 4) + 2;
    *(param.time_whole.end() - 1) = *(param.time_whole.end() - 4) + 3;

    for (int i = 0; i < num; i++) {
    param.pos_p1[i].resize(param.pos[i].size());
    param.pos_p2[i].resize(param.pos[i].size());
    param.pos_p3[i].resize(param.pos[i].size());
    }
    // for (int i = 0; i < param.time_whole.size(); i++) {
    // 	std::cout << "time: " << param.time_whole[i]<<" angle_pos1: "<< param.pos[0][i] << " angle_pos2: " << param.pos[1][i]  <<" angle_pos3: " << param.pos[2][i] << " angle_pos4: " << param.pos[3][i] << " angle_pos5: " << param.pos[4][i] << " angle_pos6: " << param.pos[5][i] <<
    // 		" vel1: " << param.vel[0][i] << " vel2: " << param.vel[1][i] << " vel3: " << param.vel[2][i] << " vel4: " << param.vel[3][i] << " vel5: " << param.vel[4][i] << " vel6: " << param.vel[5][i] << std::endl;;
    // }


    for (int i = 0; i < m->nu; i++)
    {
        m_controller.configure_actuators_pid(m,i,joint_Kpp[i],joint_Kpd[i],joint_Kpi[i],joint_Kvp[i],joint_Kvd[i]);
        m_controller.configure_actuators_vel_limit(m,"degree",i,joint_vellimit[i]);
    }

    for (int i = 0; i < m->nu; i++)
        plan.CubicSpline(param.time_whole.size(), param.time_whole.data(), param.pos[i].data(), param.vel[i].data(), param.pos_p1[i].data(), param.pos_p2[i].data(), param.pos_p3[i].data());

        ShapedImpulse shaperParam = inputshaper.shaperGenerator(ShaperType::ZVD,7,0.12);
        inputshaper.init(shaperParam.amplitudes,shaperParam.times,0.002);
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

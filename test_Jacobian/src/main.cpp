
//c++版本进行测试PID控制器
#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
//#include <math.h>
#include <algorithm> // for std::min/max
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "iostream"
#include "vector"
#include "modern_robotics.hpp"

#include "zj_mujoco.hpp"

using namespace mr;
//simulation end time
double simend = 100;

 //double qinit[6] ={0, -2.55, -0.785, 0, 0, 0};
//double qinit[6] = {0,-1,0,1,0,0};
double qinit[6] = {0,0,0,0,0,0};
//double qinit[6] = {0,-M_PI*0.5,M_PI*0.5,0,M_PI*0.5,0};
// double qinit[6] ={-0.785, -2.55, -0.785, -4.71, -1.57, 0};
double r = 0.5;
double omega = 0.5;

double x_0, y_0;

//related to writing data to a file
FILE *fid;
int loop_index = 0;
const int data_frequency = 10; //frequency at which data is written to a file


// char xmlpath[] = "../myproject/template_writeData/pendulum.xml";
// char datapath[] = "../myproject/template_writeData/data.csv";


//Change the path <template_writeData>
//Change the xml file
char path[] = "/home/wtc/ur5_test/";
char xmlfile[] = "ur5.xml";


char datafile[] = "data.csv";


// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_left1 = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// holders of one step history of time and position to calculate dertivatives
mjtNum position_history = 0;
mjtNum previous_time = 0;

// controller related variables
float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

mjvPerturb pert;




struct {
    std::optional<std::string> save_xml;
    std::optional<std::string> save_mjb;
    std::optional<std::string> print_model;
    std::optional<std::string> print_data;
    bool reset;
    bool align;
    bool copy_pose;
    bool load_from_history;
    bool load_key;
    bool save_key;
    bool zero_ctrl;
    int newperturb;
    bool select;
    mjuiState select_state;
    bool ui_update_simulation;
    bool ui_update_physics;
    bool ui_update_rendering;
    bool ui_update_joint;
    bool ui_update_ctrl;
    bool ui_remake_ctrl;
  } pending_ = {};


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);
    button_left1 =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);

    //获取当前鼠标点击位置
    if (button_left1) {
        // 获取点击位置
        double x, y;
        glfwGetCursorPos(window, &x, &y);
        // 转换到归一化坐标
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        // find geom and 3D click point, get corresponding body
        mjrRect r = pending_.select_state.rect[3];
        mjtNum pos[3];
        int selgeom, selflex, selskin;
        int sel = mjv_select(m, d, &opt,
            static_cast<mjtNum>(viewport.width) / viewport.height,
            (mjtNum)x/(viewport.width-1),
            (mjtNum)(viewport.height-1-y)/(viewport.height-1),
            &scn, pos, &selgeom, &selflex, &selskin);
        // int sel = mjv_select(m, d, &opt,
        //     static_cast<mjtNum>(r.width) / r.height,
        //     (pending_.select_state.x - r.left) / r.width,
        //     (pending_.select_state.y - r.bottom) / r.height,
        //     &scn, pos, &selgeom, &selflex, &selskin);
        //std::cout << "Selected: "<< sel << " wrist_3_link "<<mj_name2id(m, mjOBJ_BODY, "wrist_3_link")<< std::endl;
        // 如果选中机械臂末端
        if (sel == mj_name2id(m, mjOBJ_BODY, "ee_link")) {
            // 存储选中状态
            pert.active = 1;
            mju_copy3(pert.refpos, pos);
            ///std::cout << "Selected: "<< pos[0]<<"  "<<pos[1]<<"  "<<pos[2]<< std::endl;
        }
    }
    else {
        pert.active = 0;
    }


}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}




// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}




// 关节控制参数
struct JointController {
    ControlMode mode = POSITION_CONTROL;
    double target_pos = 0.0;    // 目标位置 (rad)
    double target_vel = 0.0;    // 目标速度 (rad/s)
    double target_torque = 0.0; // 目标力矩 (Nm)
    double vel_limit=0.33;       // 速度限幅 (rad)
    
    // PID 参数
    struct {
        double Kpp = 500.0;    // 位置环比例
        double Kpd = 50.0;      
        double Kpi = 10.0;      // 积分

        double Kvp = 10.0;    
        double Kvd = 10.0; 
        double Kvi = 10.0;      // 积分
        double prev_error = 0.0;   
        double integral = 0.0;
    } pid;
};


struct ServoSeriesPolynomial3Param
{
    std::vector<double>joint_set;
    std::vector<double> vel_set;
    std::vector<double> time, time_whole;
    //double scale, t, tt, ratio, stop_time;
    double scale;
    std::vector<std::vector<double>> joint_mat;
    std::vector<std::vector<double>> vel_mat;
    std::vector<std::vector<double>> pos, vel, acc;
    std::vector<std::vector<double>> pos_p1, pos_p2, pos_p3;

}param;


// 全局控制状态
struct {
  std::vector<JointController> joints; // 6个关节控制器
  int selected_joint = 0;              // 当前选中的关节 (0-5)
} g_control;

// 配置制定关节pd参数 
void configure_actuators_pid(const mjModel* m, int selected_joint, double Kpp, double Kpd, double Kvp, double Kvd) {
    auto& ctrl = g_control.joints[selected_joint];
    switch (ctrl.mode) {
      case TORQUE_CONTROL:
          break;
      case VELOCITY_CONTROL:{
          ctrl.pid.Kvp = Kvp;
          ctrl.pid.Kvd = Kvd;
          break;
          }
  
      case POSITION_CONTROL:{
          ctrl.pid.Kpp = Kpp;
          ctrl.pid.Kpd = Kpd;
          break;
          }
      }
  }

// 配置执行器控制模式
void configure_actuators_mode(const mjModel* m, ControlMode mode) {
  for (int i = 0; i < m->nu; i++) {
      switch (mode) {
          case TORQUE_CONTROL:
              g_control.joints[i].mode = TORQUE_CONTROL;
              break;
          case VELOCITY_CONTROL:
              g_control.joints[i].mode = VELOCITY_CONTROL;
              break;
          case POSITION_CONTROL:
              g_control.joints[i].mode = POSITION_CONTROL;
              break;
      }
  }
}

void configure_actuators_vel_limit(const mjModel* m, const std::string& vel_type,int selected_joint, double vel_limit) {
    auto& joint = g_control.joints[selected_joint];
    if (vel_type=="RPM")
    {
        joint.vel_limit = vel_limit/60*2*M_PI;//RPM->rad
    }else if(vel_type=="rad"){
        joint.vel_limit = vel_limit;//rad->rad
    }else if(vel_type=="degree"){
        joint.vel_limit = vel_limit/360*2*M_PI;//degree->rad
    }

}


// 逆动力学控制
//阻抗导纳控制  https://zhuanlan.zhihu.com/p/705884402
// double ee_pm[16]{-0.706718,-0.694522 ,0.134867 ,-0.367861,
//     -0.707495, 0.693969 ,-0.133634, 0.521961 ,
//     -0.000781843, -0.189859 ,-0.981811, 0.18795 ,
//     0 ,0 ,0 ,1 };

double ee_pm[16]{-1.11022e-16 ,0, 1 ,0.47455 ,
1 ,0 ,1.11022e-16 ,0.10915 ,
0, 1 ,0 ,0.419509 ,
0 ,0 ,0 ,1 };

// 控制器回调
void controller(const mjModel* m, mjData* d) {
    double time = d->time;
    //mj_forward(m,d);
    //正动力学计算
    //mj_forward(m,d);
    ee_pm[11]-=0.001;
    //FK
    // std::cout<<"fk"<<std::endl;
    //  Eigen::MatrixXd ans=ur5_FK(m,d);
    // double ee_pm[16]{ 0 };
    // for (int i = 0; i < 4; i++) {
    //     for (int  j= 0; j < 4; j++)
    //     {
    //         ee_pm[i+j] = ans(i,j);
    //         std::cout << ans(i,j) << " ";
    //     }
    //     std::cout<<std::endl;
    // }
    // //Ik
    
    Eigen::VectorXd thetaList=InverseKinematic(ee_pm,d);
    double target_pos[6] = {0, 0, 0, 0, 0, 0}; 
    double target_vel[6] = {0, 0, 0, 0, 0, 0}; 
    double target_acc[6] = {0, 0, 0, 0, 0, 0}; 



    //针对零力拖动逆动力学计算
    // mju_copy(d->qvel, target_vel, m->nv);
    // mju_copy(d->qacc, target_acc, m->nv);
    for (int i = 0; i < m->nu; i++) {
        auto& ctrl = g_control.joints[i];
        switch (ctrl.mode) {
            case TORQUE_CONTROL:
                d->ctrl[i] = ctrl.target_torque;
                //d->ctrl[i] = ctrl.target_torque+d->qfrc_bias[i];
                //std::cout<<time<<"  "<<d->ctrl[i]<<"  "<<d->qpos[i]<<"  "<<d->actuator_force[i]<<std::endl;
                std::cout<<" "<<d->qvel[i]*360/M_PI<<"  bias "<<d->qfrc_bias[i]<<" actuator "<<d->qfrc_actuator[i]<<" passive "<<d->qfrc_passive[i]<<" applied "<<d->qfrc_applied[i]<<" constraint "<<d->qfrc_constraint[i]<<" inverse "<<d->qfrc_inverse[i]<<" "<<d->actuator_force[i]<<std::endl;
                break;
            case VELOCITY_CONTROL: {
                double error = ctrl.target_vel - d->qvel[i];
                double derivative = (error - ctrl.pid.prev_error) / m->opt.timestep;
                ctrl.pid.prev_error = error;
                d->ctrl[i] = ctrl.pid.Kvp * error + ctrl.pid.Kvd * derivative;
                break;
                }
            case POSITION_CONTROL: {
                //ctrl.target_pos=0 + 0.5*sin(2*M_PI*0.5*time);/* code */
                ctrl.target_pos=qinit[i];
                // ctrl.target_vel=0 + 2*M_PI*0.5*0.5*cos(2*M_PI*0.5*time);/* code */
                //ctrl.target_pos=thetaList[i];
                double pos_error = ctrl.target_pos - d->qpos[i];
                double pos_control=ctrl.pid.Kpp * pos_error;
                //pos_control=std::clamp(pos_control,-ctrl.vel_limit,ctrl.vel_limit);
                //tor_limit
                double vel_error = ctrl.target_vel - d->qvel[i];
                double vel_control=ctrl.pid.Kpd* vel_error;
                //pd控制
                //d->ctrl[i] = pos_control + vel_control+d->qfrc_bias[i];
                //pd控制+Feedforward
                //vel_control=std::clamp(vel_control,m->actuator_ctrlrange[2 * i],m->actuator_ctrlrange[2 * i + 1]);
                target_pos[i]=ctrl.target_pos;
                target_vel[i]=0;
                target_acc[i]=0;
                d->ctrl[i] = pos_control + vel_control;

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
                double vel_freedforward=pos_error;
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
                //std::cout<<"  "<<" "<<vel_freedforward<<" "<<d->qvel[i]*360/M_PI<<"  "<<(ctrl.target_pos-d->qpos[i])*360/M_PI<<"  bias "<<d->qfrc_bias[i]<<" actuator "<<d->qfrc_actuator[i]<<" passive "<<d->qfrc_passive[i]<<" applied "<<d->qfrc_applied[i]<<" constraint "<<d->qfrc_constraint[i]<<" inverse "<<d->qfrc_inverse[i]<<" "<<d->actuator_force[i]<<std::endl;
                
                break;
            }
        }
    }
    

    target_vel[0]=0.5;
    //设置模型的位置、速度、加速度，进行逆动力学计算
    mju_copy(d->qpos, target_pos, m->nq);
    //零力拖动只计算重力补偿
    mju_copy(d->qvel, target_vel, m->nv);
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



    // int ee_id = mj_name2id(m, mjOBJ_BODY, "ee_link");
    //     if(ee_id != -1) {
    //         double vel[6];
    //         mj_objectVelocity(m, d, mjOBJ_BODY, ee_id, vel, 0);
    //         std::cout<<" vel 111:"<<std::endl;
    //         for (size_t i = 0; i < 6; i++)
    //         {
    //             std::cout<<"  "<<vel[i];
    //         }
    //         std::cout<<std::endl;
    //         // 数据结构：
    //         // vel[0-2] 角速度(rad/s)
    //         // vel[3-5] 线速度(m/s)
    //     }


    // std::cout<<"sensor vel: "<<d->sensordata[16]<<"  "<<d->sensordata[16]<<"  "<<d->sensordata[18]<<std::endl;
    // std::cout<<"joint_vel: "<<d->qvel[0]<<"  "<<d->qvel[1]<<"  "<<d->qvel[2]<<" "<<d->qvel[3]<<"  "<<d->qvel[4]<<"  "<<d->qvel[5]<<std::endl;

    // Eigen::MatrixXd vel1111 = Eigen::MatrixXd::Zero(6, 1);
    // for (int  j= 0; j < 6; j++)
    // {
    //     vel1111(j,0) = d->qvel[j];

    // }
    // Eigen::MatrixXd JacobianSpace=ur5_JacobianSpace(m,d);
    // auto tor1=JacobianSpace*vel1111;
    // std::cout<<" JacobianSpace "<<std::endl;
    // for (size_t i = 0; i < 6; i++)
    // {
    //     std::cout<<"  "<<tor1(i,0);
    // }
    // std::cout<<std::endl;



    // std::cout<<std::endl;
}
// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{

}

void init_controller(const mjModel* m, mjData* d)
{
  mj_forward(m,d);
  g_control.joints.resize(6);
  configure_actuators_mode(m, POSITION_CONTROL);
  double joint_Kpp[]={100,100,100,100,100,100};
  double joint_Kpd[]={30,30,30,30,30,30};

  double joint_Kvp[]={100,100,100,200,200,20};
  double joint_Kvd[]={10,10,10,10,10,10};
  double joint_vellimit[]={120,120,120,120,120,120};

    for (int i = 0; i < m->nu; i++)
    {
        configure_actuators_pid(m,i,joint_Kpp[i],joint_Kpd[i],joint_Kvp[i],joint_Kvd[i]);
        configure_actuators_vel_limit(m,"degree",i,joint_vellimit[i]);
    }
}
void disable_constraints(mjModel* m, mjData* d) {
    // 方法 1：全局禁用约束
    m->opt.disableflags |= mjDSBL_CONSTRAINT;
}


//************************
// main function
int main(int argc, const char** argv)
{


    char xmlpath[100]={};
    char datapath[100]={};

    strcat(xmlpath,path);
    strcat(xmlpath,xmlfile);

    strcat(datapath,path);
    strcat(datapath,datafile);


    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if( argc<2 )
        m = mj_loadXML(xmlpath, 0, error, 1000);

    else
        if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
      mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);


    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");
    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    double arr_view[] = {1, 1, 2, 0.000000, 0.000000, 0.000000};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

    for (size_t i = 0; i < 6; i++)
    {
      d->qpos[i] = qinit[i];
      d->qvel[i]=0;
      d->qacc[i]=0;
    }


    // // 获取末端执行器 ID
    // int ee_body_id = mj_name2id(m, mjOBJ_BODY, "ee_link");
    // if (ee_body_id == -1) {
    //     std::cerr << "未找到 ee_link" << std::endl;
    //     return 1;
    // }



    // 方法1：禁用所有接触约束,使得constraint为0
    disable_constraints(m,d);
    // 初始化控制器
    init_controller(m,d);
    // 设置控制器
    mjcb_control = controller;



    bool first = true;
    //fid = fopen(datapath,"w");
    //init_save_data();

    // use the first while condition if you want to simulate for a period.
    while( !glfwWindowShouldClose(window))
    {
        if (!pert.active) {
            first=false;
            // 获取鼠标位移量
            // double dx, dy;
            // glfwGetCursorPos(window, &dx, &dy);
            // mjtMouse action;
            // mjrRect r = pending_.select_state.rect[3];
            // if (pending_.select_state.right) {
            //   action = pending_.select_state.shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
            // } else if (pending_.select_state.left) {
            //   action = pending_.select_state.shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
            // } else {
            //   action = mjMOUSE_ZOOM;
            // }
            // // 转换为3D空间位移
            mjtNum force[3] = {0};
            mjtNum torque[3] = {0};
            // mjv_movePerturb(m, d, action, dx / r.height, -dy / r.height,&scn, &pert);
            // // mjv_movePerturb(m, d,action, dx, dy, &scn, &pert);
 
            // 设置外力参数（单位：牛顿）
            force[0] =  0* 50;  // X方向
            force[1] = 1 * 50;  // Y方向 
            force[2] = 0* 50;  // Z方向
            pert.refpos[0]=d->sensordata[12];
            pert.refpos[1]=d->sensordata[13];
            pert.refpos[2]=d->sensordata[14];
            //std::cout << "Selected1111: "<< pert.refpos[0]<<"  "<<pert.refpos[1]<<"  "<<pert.refpos[2]<< std::endl;
            

            // 施加外力到选定刚体
            // mj_applyFT(m, d, force, torque,pert.refpos, 
            //     mj_name2id(m, mjOBJ_BODY, "wrist_3_link"), 
            //     d->qfrc_applied);
            std::cout<<" joint pose "<<std::endl;
            for (size_t i = 0; i < 6; i++)
            {
                std::cout<<"  "<<d->qpos[i];
            }
            std::cout<<std::endl;

            std::cout<<" joint vel "<<std::endl;
            for (size_t i = 0; i < 6; i++)
            {
                std::cout<<"  "<<d->qvel[i];
            }
            std::cout<<std::endl;



            Eigen::MatrixXd force_in_body = Eigen::MatrixXd::Zero(6, 1);
            
            for (int  j= 0; j < 3; j++)
            {
                force_in_body(j,0) = force[j];
            }

            //获取ID号
            // 末端执行器在xml中名字为link6，查找名字为"link6"的body的编号
            int ee_id = mj_name2id(m, mjOBJ_BODY, "ee_link");
            int ee_id1 = mj_name2id(m, mjOBJ_SITE, "endpos");
            // // 1. mujoco 计算雅可比矩阵
            // 计算总dof数量
            int total_joints = m->njnt;
            int free_joints = 0;
            for (int i = 0; i < total_joints; ++i)
                if (m->jnt_type[i] == mjJNT_FREE)
                        free_joints++;
            int dof = total_joints + 5 * free_joints;
            //std::cout<<" dof: "<<dof<<" nv "<<m->nv<<std::endl;

            //方法一可以
            // 定义jacp、jacr以及坐标系参考点位
            // std::vector<mjtNum> jacp(3*dof, 0);
            // std::vector<mjtNum> jacr(3*dof, 0);
            // // 获取参考点坐标（末端执行器位置）
            // mjtNum ref_point[3];
            // mju_copy3(ref_point, d->xpos + ee_id*3);  // 复制body位置
            // // 计算雅可比矩阵
            // mj_forward(m,d);
            // mj_jac(m, d, jacp.data(), jacr.data(), ref_point, ee_id);
            // Eigen::MatrixXd jac(6, m->nv);
            // for (int i = 0; i < 3; ++i){
            //     for (int j = 0; j < 6; ++j)
            //     {
            //         jac(i, j) = jacp[i * dof + j];
            //         jac(i + 3, j) = jacr[i * dof + j];
            //     }
            // }
            Eigen::VectorXd qdot = Eigen::Map<Eigen::VectorXd>(d->qvel, m->nv);
            // Eigen::VectorXd cart_vel = jac * qdot;
            // // 输出结果（前3个为线速度，后3个为角速度）
            //  std::cout << "笛卡尔速度 (m/s, rad/s):\n" << cart_vel.transpose()<< std::endl;
            //  std::cout << "jac:\n" << jac<< std::endl;


            //方法二可以
            // 提取线速度和角速度雅可比
            Eigen::MatrixXd jac1(6, m->nv);
            std::vector<mjtNum> jacp1(3*dof, 0);
            std::vector<mjtNum> jacr1(3*dof, 0);
            mj_jacSite(m, d,jacp1.data(),jacr1.data(),ee_id1);
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 6; ++j)
                    {
                        jac1(i, j) = jacp1[i * dof + j];
                        jac1(i + 3, j) = jacr1[i * dof + j];
                    }
            }
            //计算笛卡尔速度 (假设已有关节速度 q_dot)
            Eigen::VectorXd cart_vel1 = jac1 * qdot;
            // 输出结果（前3个为线速度，后3个为角速度）
            std::cout << "笛卡尔速度11 (m/s, rad/s):\n" << cart_vel1.transpose()<< std::endl;
            //std::cout << "jac1:\n" << jac1<< std::endl;



            // 获取末端姿态矩阵
            // mjtNum end_rot[9];
            // mju_quat2Mat(end_rot, d->xquat + 4*ee_id1); // xquat为四元数
            // 将雅可比结果转换到世界坐标系
            // Eigen::Matrix3d R_world_end = Eigen::Map<Eigen::Matrix3d>(end_rot).transpose();
            // Eigen::Vector3d vel_world = R_world_end * jac1.block<3,6>(0,0) * qdot;
            // std::cout << "笛卡尔vel_world速度11 (m/s, rad/s):\n" << vel_world<< std::endl;

            //两种观测均可
             if(ee_id != -1) {
                 double vel[6];
                 mj_objectVelocity(m, d, mjOBJ_BODY, ee_id, vel, 0);
                 std::cout<<"mj_objectVelocity :"<<std::endl;
                 for (size_t i = 0; i < 6; i++)
                 {
                     std::cout<<"  "<<vel[i];
                 }
                 std::cout<<std::endl;
                 // 数据结构：
                 // vel[0-2] 角速度(rad/s)
                 // vel[3-5] 线速度(m/s)
             }


            if(ee_id1 != -1) {
                double vel[6];
                mj_objectVelocity(m, d, mjOBJ_SITE, ee_id1, vel, 0);
                std::cout<<"mj_objectVelocity 2 :"<<std::endl;
                for (size_t i = 0; i < 6; i++)
                {
                    std::cout<<"  "<<vel[i];
                }
                std::cout<<std::endl;
                // 数据结构：
                // vel[0-2] 角速度(rad/s)
                // vel[3-5] 线速度(m/s)
            }
            //std::cout<<"joint_vel: "<<d->qvel[0]<<"  "<<d->qvel[1]<<"  "<<d->qvel[2]<<" "<<d->qvel[3]<<"  "<<d->qvel[4]<<"  "<<d->qvel[5]<<std::endl;
            
            
            std::cout<<"sensor pos: "<<d->sensordata[12]<<"  "<<d->sensordata[13]<<"  "<<d->sensordata[14]<<std::endl;
            std::cout<<"framelinvel: "<<d->sensordata[15]<<"  "<<d->sensordata[16]<<"  "<<d->sensordata[17]<<std::endl;
            
            
             // std::cout<<"frameangvel: "<<d->sensordata[18]<<"  "<<d->sensordata[19]<<"  "<<d->sensordata[20]<<std::endl;
            // std::cout<<"frameangvel: "<<d->sensordata[21]<<"  "<<d->sensordata[22]<<"  "<<d->sensordata[23]<<std::endl;

            // Eigen::MatrixXd vel_joint = Eigen::MatrixXd::Zero(6, 1);
            // for (int  j= 0; j < 6; j++)
            // {
            //     vel_joint(j,0) = d->qvel[j];

            // }
            Eigen::MatrixXd JacobianSpace=ur5_JacobianSpace(m,d);
            std::cout << "JacobianSpace :\n" << JacobianSpace<< std::endl;
            auto tor1=JacobianSpace*qdot;
            std::cout<<" zj JacobianSpace linvel "<<std::endl;
            for (size_t i = 0; i < 6; i++)
            {
                std::cout<<"  "<<tor1(i,0);
            }
            std::cout<<std::endl;


            // // 2. 矩阵转置运算
            // auto tor_mujoco = J.transpose() * force_in_body;
            // // auto force_in_world=ad_ret.transpose()*force_in_body;
            // std::cout<<"tor_mujoco :"<<std::endl;
            // for (int  j= 0; j < 6; j++)
            // {
            //     std::cout<<tor_mujoco(j,0)<<" ";
            //     //d->qfrc_applied[j]=tor_mujoco(j,0);
            // }
            std::cout<<std::endl;


////////////////////////////////////////////////////
            //测试力雅可比d->qpos[0]

            // std::cout<<" qfrc_applied "<<std::endl;
            // for (size_t i = 0; i < 6; i++)
            // {
            //     std::cout<<"  "<<d->qfrc_applied[i];
            //     // force_in_world(i,0)=FT_in_world[i];
            //     // force_in_body(i,0)=FT_in_sensor[i];
            // }
            // std::cout<<std::endl;

            // // 1. 计算雅可比矩阵
            // Eigen::MatrixXd force_in_body = Eigen::MatrixXd::Zero(6, 1);
            // for (int  j= 0; j < 3; j++)
            // {
            //     force_in_body(j,0) = force[j];
            // }
            // int ee_body_id = mj_name2id(m, mjOBJ_BODY, "ee_link");
            // Eigen::MatrixXd J(6, m->nv);
            // mj_jacSite(m, d, J.data(), nullptr, ee_body_id);
            



            // // 2. 矩阵转置运算
            // auto tor_mujoco = J.transpose() * force_in_body;
            // // auto force_in_world=ad_ret.transpose()*force_in_body;
            // std::cout<<"tor_mujoco :"<<std::endl;
            // for (int  j= 0; j < 6; j++)
            // {
            //     std::cout<<tor_mujoco(j,0)<<" ";
            //     //d->qfrc_applied[j]=tor_mujoco(j,0);
            // }
            // std::cout<<std::endl;


            // auto tor2222222=J*vel1111;

            // std::cout<<"tor2222222 :"<<std::endl;
            // for (int  j= 0; j < 6; j++)
            // {
            //     std::cout<<tor2222222(j,0)<<" ";
            //     //d->qfrc_applied[j]=tor_mujoco(j,0);
            // }
            // std::cout<<std::endl;


            // Eigen::MatrixXd JacobianSpace=ur5_JacobianBody(m,d);
            // auto tor2=JacobianSpace.transpose()*force_in_body;
            // std::cout<<" JacobianSpace "<<std::endl;
            // for (size_t i = 0; i < 6; i++)
            // {
            //     std::cout<<"  "<<tor2(i,0);
            // }
            // std::cout<<std::endl;

            std::cout<<std::endl;


                // 步骤1：获取雅可比矩阵

            
            // 步骤2：计算伪逆（带阻尼最小二乘）
            // double lambda = 1e-6; // 阻尼系数
            // Eigen::MatrixXd J_T = J.transpose();
            // Eigen::MatrixXd JJT = J * J_T;
            // JJT.diagonal().array() += lambda;
            // Eigen::MatrixXd J_pseudo = J_T * JJT.inverse();
            
            // // 步骤3：计算笛卡尔力
            // Eigen::VectorXd tau(m->nv);
            // for(int i=0; i<m->nv; ++i) tau[i] = d->ctrl[i];
            // Eigen::VectorXd F = J_pseudo * tau;
            
            // // 输出笛卡尔力（前3维力，后3维力矩）
            // mju_printVec("Cartesian Force", F.data(), 6);

            // //////////测试静力学 f->tor
            // double ee_pm[16]{ 0 };
            // Eigen::MatrixXd force_in_body = Eigen::MatrixXd::Zero(6, 1);
            // double FT_in_sensor[6]{0};
            // double FT_in_world[6]{0};
            // std::cout<<"fk"<<std::endl;
            // Eigen::MatrixXd Tab=ur5_FK(m,d);
            // for (int i = 0; i < 4; i++) {
            //     for (int  j= 0; j < 4; j++)
            //     {

            //         ee_pm[i+j] = Tab(i,j);
            //         std::cout << Tab(i,j) << " ";
            //     }
            //     std::cout<<std::endl;
            // }
            // // //Ik
            // for (int  j= 0; j < 3; j++)
            // {
            //     force_in_body(j,0) = force[j];
            //     FT_in_sensor[j]= force[j];
            // }
            // std::cout<<"force_in_body:"<<std::endl;
            // for (int  j= 0; j < 6; j++)
            // {

            //     std::cout<<force_in_body(j,0)<<" ";
            // }
            // std::cout<<std::endl;

			// s_pm_dot_v3(ee_pm, FT_in_sensor, FT_in_world);
			// s_pm_dot_v3(ee_pm, FT_in_sensor + 3, FT_in_world + 3);


            // std::cout<<"FT_in_world:"<<std::endl;
            // for (int  j= 0; j < 6; j++)
            // {

            //     std::cout<<FT_in_world[j]<<" ";
            // }
            // std::cout<<std::endl;



            // //Tba=Tab-
            // //力旋量 Fa=[AdTba]T*Fb
            // Eigen::MatrixXd Tba = Tab.inverse();

            // auto ad_ret= Adjoint(Tba);
            // auto force_in_world=ad_ret.transpose()*force_in_body;


            // std::cout<<"FT_in_world  1:"<<std::endl;
            // for (int  j= 0; j < 6; j++)
            // {
            //     std::cout<<force_in_world(j,0)<<" ";
            // }
            // std::cout<<std::endl;



            // std::cout<<" qfrc_applied "<<std::endl;
            // for (size_t i = 0; i < 6; i++)
            // {
            //     std::cout<<"  "<<d->qfrc_applied[i];
            //     // force_in_world(i,0)=FT_in_world[i];
            //     // force_in_body(i,0)=FT_in_sensor[i];
            // }
            // std::cout<<std::endl;

            // //force in word
            // //tor=JacobianSpace.transpose()*force1;
            // Eigen::MatrixXd JacobianSpace=ur5_JacobianSpace(m,d);
            // auto tor1=JacobianSpace.transpose()*force_in_body;
            // std::cout<<" tor1"<<std::endl;
            // for (size_t i = 0; i < 6; i++)
            // {
            //     std::cout<<"  "<<tor1(i,0);
            //     d->qfrc_applied[i]=tor1(i,0);
            // }
            // std::cout<<std::endl;


            // //force in body
            // //tor=JacobianSpace.transpose()*force1;
            // Eigen::MatrixXd JacobianSpace1=ur5_JacobianBody(m,d);
            // auto tor2=JacobianSpace1.transpose()*force_in_world;
            // std::cout<<" tor2 "<<std::endl;
            // for (size_t i = 0; i < 6; i++)
            // {
            //     std::cout<<"  "<<tor2(i,0);
            //     //d->qfrc_applied[i]=tor1(i,0);
            // }
            // std::cout<<std::endl;

            // /////////测试静力学 tor->f
            // auto ans111=JacobianSpace.transpose();
            // std::cout<<" tor3"<<std::endl;
            // Eigen::MatrixXd force1111=pinv(ans111)*tor1;

            // for (size_t i = 0; i < 6; i++)
            // {
            //     std::cout<<"  "<<force1111(i,0);
            //     //d->qfrc_applied[i]=tor1(i,0);
            // }
            // std::cout<<std::endl;
        }
        else{
            mju_zero(d->qfrc_applied, 6*m->nbody);
        }



        // 仿真一步
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
        {
            mj_step(m, d);
            mj_forward(m,d);
        }
        //if (d->time>=simend)
        if (d->time>=simend)
        {
           //fclose(fid);
           break;
         }

       // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

          // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);


    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}

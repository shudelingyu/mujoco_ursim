
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
double simend = 20;
double qinit[6] = {0,0,0,0,0,0};
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



// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
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

// 控制器回调
void controller(const mjModel* m, mjData* d) {
    double time = d->time;
    //正动力学计算
    //mj_forward(m,d);
    //FK
    Eigen::MatrixXd ans=ur5_FK(m,d);
    double ee_pm[16]{ 0 };
    for (int i = 0; i < 4; i++) {
        for (int  j= 0; j < 4; j++)
        {
            ee_pm[i+j] = ans(i,j);
            //std::cout << ans(i,j) << " ";
        }
        //std::cout<<std::endl;
    }
    //Ik
    Eigen::VectorXd thetaList=InverseKinematic(ee_pm,d);



    // 逆动力学求解
    //mj_inverse(m,d);
    // double dense_M[m->nv*m->nv] = {0};
    // mj_fullM(m,dense_M, d->qM);

    double target_pos[6] = {0, 0, 0, 0, 0, 0}; 
    double target_vel[6] = {0, 0, 0, 0, 0, 0}; 
    double target_acc[6] = {0, 0, 0, 0, 0, 0}; 

    for (int i = 0; i < m->nu; i++) {
        auto& ctrl = g_control.joints[i];

        ctrl.target_pos= CubicSpline_at(param.time_whole.size(), param.time_whole.data(), param.pos[i].data(), param.vel[i].data(), param.pos_p1[i].data(), param.pos_p2[i].data(), param.pos_p3[i].data(), time);
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
                std::cout<<"  "<<" "<<vel_freedforward<<" "<<d->qvel[i]*360/M_PI<<"  "<<(ctrl.target_pos-d->qpos[i])*360/M_PI<<"  bias "<<d->qfrc_bias[i]<<" actuator "<<d->qfrc_actuator[i]<<" passive "<<d->qfrc_passive[i]<<" applied "<<d->qfrc_applied[i]<<" constraint "<<d->qfrc_constraint[i]<<" inverse "<<d->qfrc_inverse[i]<<" "<<d->actuator_force[i]<<std::endl;
                break;
            }
        }
    }
    std::cout<<std::endl;


    //设置模型的位置、速度、加速度，进行逆动力学计算
    mju_copy(d->qpos, target_pos, m->nq);
    mju_copy(d->qvel, target_vel, m->nv);
    mju_copy(d->qacc, target_acc, m->nv);

    // 逆动力学控制，计算关节力矩
    mj_inverse(m,d);
    for (size_t i = 0; i < 6; i++)
    {
        //d->ctrl[i] = d->ctrl[i]+d->qfrc_inverse[i];
        d->ctrl[i] = d->ctrl[i]+d->qfrc_bias[i];
        //d->ctrl[i] = d->ctrl[i];
        //d->ctrl[i] = d->qfrc_bias[i];
    }
    
}


// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // // backspace: reset simulation
    // if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    // {
    //     mj_resetData(m, d);
    //     mj_forward(m, d);
    // }
}




void init_controller(const mjModel* m, mjData* d)
{
  mj_forward(m,d);
  g_control.joints.resize(6);
  configure_actuators_mode(m, POSITION_CONTROL);
  double joint_Kpp[]={100,100,100,100,100,100};
  double joint_Kpd[]={30,30,30,30,10,10};

  double joint_Kvp[]={100,100,100,200,200,20};
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
            std::cout<<" "<< target_pos[j];
        }
        std::cout<<std::endl;
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
        configure_actuators_pid(m,i,joint_Kpp[i],joint_Kpd[i],joint_Kvp[i],joint_Kvd[i]);
        configure_actuators_vel_limit(m,"degree",i,joint_vellimit[i]);
    }

    for (int i = 0; i < m->nu; i++)
        CubicSpline(param.time_whole.size(), param.time_whole.data(), param.pos[i].data(), param.vel[i].data(), param.pos_p1[i].data(), param.pos_p2[i].data(), param.pos_p3[i].data());

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

    // 方法1：禁用所有接触约束,使得constraint为0
    disable_constraints(m,d);


    // 初始化控制器
    init_controller(m,d);
    // 设置控制器
    mjcb_control = controller;




    //fid = fopen(datapath,"w");
    //init_save_data();
    
    // use the first while condition if you want to simulate for a period.
    while( !glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
        {
            mj_step(m, d);
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

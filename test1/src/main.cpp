
//c++版本进行测试PID控制器
#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
//#include <math.h>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "iostream"
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

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

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


//****************************
//This function is called once and is used to get the headers
void init_save_data()
{
  //write name of the variable here (header)
   fprintf(fid,"t, ");
   fprintf(fid,"x, y ");

   //Don't remove the newline
   fprintf(fid,"\n");
}

//***************************
//This function is called at a set frequency, put data here
// void save_data(const mjModel* m, mjData* d)
// {
//   //data here should correspond to headers in init_save_data()
//   //seperate data by a space %f followed by space
//   fprintf(fid,"%f, ",d->time);
//   fprintf(fid,"%f, %f ",d->sensordata[0],d->sensordata[2]);

//   //Don't remove the newline
//   fprintf(fid,"\n");
// }

/******************************/
void set_torque_control(const mjModel* m,int actuator_no,int flag)
{
  m->actuator_gainprm[actuator_no*3, 0] = 0;
  m->actuator_biasprm[actuator_no*3, 1] = 0;
  m->actuator_gainprm[actuator_no*3+1, 0] = 0;
  m->actuator_biasprm[actuator_no*3+1, 2] = 0;
}
/******************************/


/******************************/
void set_position_servo(const mjModel* m,int actuator_no,double kp,double kv)
{
  m->actuator_gainprm[actuator_no*3, 0] = kp;
  m->actuator_biasprm[actuator_no*3, 1] = -kp;
  m->actuator_gainprm[actuator_no*3+1, 0] = kv;
  m->actuator_biasprm[actuator_no*3+1, 2] = -kv;
}
/******************************/

/******************************/
void set_velocity_servo(const mjModel* m,int actuator_no,double kv)
{
  m->actuator_gainprm[actuator_no*3, 0] = 0;
  m->actuator_biasprm[actuator_no*3, 1] = 0;
  m->actuator_gainprm[actuator_no*3+1, 0] = kv;
  m->actuator_biasprm[actuator_no*3+1, 2] = -kv;
}
/******************************/

//**************************
void init_controller(const mjModel* m, mjData* d)
{
  mj_forward(m,d);
  for (size_t i = 0; i < 6; i++)
  {
    //set_position_servo(m,i,10,1); //set pservo1 to 0

    //set_velocity_servo(m,i,0.1);
    //set_torque_control1(m, i);
  }
  
}



void set_torque_control1(const mjModel* m, int actuator_no)
{
  m->actuator_gainprm[actuator_no+0] = 0;
  m->actuator_biasprm[actuator_no+1] = 0;
}
//***********************************


//*************************************
void set_velocity_servo1(const mjModel* m, int actuator_no, double kv)
{
  m->actuator_gainprm[10*actuator_no+0] = kv;
  m->actuator_biasprm[10*actuator_no+2] = -kv;
}
//***********************************


//*************************************
void set_position_servo1(const mjModel* m, int actuator_no, double kp)
{
  m->actuator_gainprm[actuator_no+0] = kp;
  m->actuator_biasprm[actuator_no+1] = -kp;
}

// for(int j=0; j<6; j++){
//   double error = target[j] - qpos[j];    // 位置误差
//   double tau = kp[j]*error - kd[j]*qvel[j]; // 计算控制力矩
//   data->ctrl[j] = tau;                   // 写入控制信号
// }
//**************************
void mycontroller(const mjModel* m, mjData* d)
{

  // 生成sin波轨迹
  double time = d->time;
  double target = 0;

  //pos_controlled
  // for (size_t i = 0; i < 6; i++)
  // {
  //   set_position_servo1(m, i, 200);
  //   d->ctrl[i] = 0 + 0.5*sin(2*M_PI*0.5*time);
  //   //d->qpos[i]= 0 + 0.5*sin(2*M_PI*0.5*time);
  //   d->qvel[i]=0;
  //   d->qacc[i]=0;
  //   //std::cout<<"       "<<d->qfrc_bias[i];
  //   std::cout<<time<<"       "<<d->ctrl[i]<<"   "<<d->qpos[i]<<"  "<<d->actuator_force[i];
  // }
  // std::cout<<std::endl;

  //vel_controlled
  // double kp[6] = {10,10,10,10,10,10};
  // double kd[6] = {1,1,1,1,1,1};
  // double limit=100;
  // for (size_t i = 0; i < 6; i++)
  // {
  //   //d->ctrl[2*i] = 0 + 0.5*sin(2*M_PI*0.5*time);
  //   target= 0 + 0.5*sin(2*M_PI*0.5*time);
  //   double error = target -d->qpos[i];    // 位置误差
  //   double tau = kp[i]*error - kd[i]*d->qvel[i]; // 计算控制力矩
  //   d->ctrl[i] = tau;                   // 写入控制信号
  //   d->ctrl[i]= std::min(d->ctrl[i], limit);
  //   d->ctrl[i] =std::max(d->ctrl[i], -limit);
  //   // d->qvel[i]=0;
  //   // d->qacc[i]=0;
  //   //d->qfrc_applied[i]= tau;   
  //   std::cout<<"       "<<d->ctrl[i]<<"   "<<d->actuator_force[i];
  //   //std::cout<<"       "<<d->ctrl[2*i]<<"   "<<d->qpos[i];
  // }
  // std::cout<<std::endl;




  //double targetpos[6] = {0,100,10,10,0,0};

  //pos_controlled
  // double targetpos[6] = {0,1.57,1.57,1,0,0};
  // for (size_t i = 0; i < 6; i++)
  // {
  //   d->ctrl[3*i]=targetpos[i];
  //   d->ctrl[3*i+1]=0;
  //   d->ctrl[3*i+2]=0;
  // }
  
  //   //vel_controlled
  // double targetvel[6] = {0,1,1,1,0,0};
  // for (size_t i = 0; i < 6; i++)
  // {
  //   d->ctrl[3*i]=0;
  //   d->ctrl[3*i+1]=targetvel[i];
  //   d->ctrl[3*i+2]=0;
  // }


  double kp[6] = {10,10,10,10,10,10};
  double kd[6] = {1,1,1,1,1,1};
  double limit=100;
  for (size_t i = 0; i < 6; i++)
  {
    //d->ctrl[2*i] = 0 + 0.5*sin(2*M_PI*0.5*time);
    target= 0 + 0.5*sin(2*M_PI*0.5*time);
    double error = target -d->qpos[i];    // 位置误差
    double tau = kp[i]*error - kd[i]*d->qvel[i]; // 计算控制力矩
    d->ctrl[i] = tau;                   // 写入控制信号
    d->ctrl[i]= std::min(d->ctrl[i], limit);
    d->ctrl[i] =std::max(d->ctrl[i], -limit);
    // d->qvel[i]=0;
    // d->qacc[i]=0;
    //d->qfrc_applied[i]= tau;   
    // std::cout<<"       "<<d->ctrl[i]<<"   "<<d->actuator_force[i];
    //std::cout<<"       "<<d->ctrl[2*i]<<"   "<<d->qpos[i];
  }
  // std::cout<<std::endl;


  // for (size_t i = 0; i < 6; i++)
  // {
  //   /* code */
  //   double limit=100;
  //   d->ctrl[3*i+2] = 30.0 * (1.57 - d->sensordata[i*2]) - 5.0 * d->sensordata[i*2+1];
  //   d->ctrl[3*i+2]= std::min(d->ctrl[3*i+2], limit);
  //   d->ctrl[3*i+2] =std::max(d->ctrl[3*i+2], -limit);
  //   std::cout<<"   "<<d->qfrc_applied[i];

  //   //d->qfrc_applied[i]=10*(1.57-d->qpos[i])-1*d->qvel[i];
  //   // d->qfrc_applied[i] = std::min(d->qfrc_applied[i], limit);
  //   // d->qfrc_applied[i] = std::max(d->qfrc_applied[i], -limit);
  //   // std::cout<<"   "<<d->qfrc_applied[i];
  // }
  // std::cout<<std::endl;
  
}


//************************
// main function
int main(int argc, const char** argv)
{

    std::cout<<"start main!!!!!!!!"<<std::endl;
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
    init_controller(m,d);

    // install control callback
    mjcb_control = mycontroller;


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
    std::cout<<"out main!!!!!!!!"<<std::endl;
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

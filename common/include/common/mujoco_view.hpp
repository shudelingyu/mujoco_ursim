#include <GLFW/glfw3.h>
// #include <glad/glad.h>
#include <mujoco/mujoco.h>
#include "mujoco_util.hpp"
#include "controller.hpp"
#include <iostream>
namespace common
{
    class MuJoCoViewer
    {
    public:
        MuJoCoViewer(MujocoContext *mjWrapper);
        ~MuJoCoViewer();
        void run(); 

    private:
        // 静态回调函数
        static void keyboardCallback(GLFWwindow *window, int key, int scancode, int action, int mods);
        static void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods);
        static void mouseMoveCallback(GLFWwindow *window, double xpos, double ypos);
        static void scrollCallback(GLFWwindow *window, double xoffset, double yoffset);
        //绘制关节角
        void drawJointAngles(const mjrRect& viewport);
    private:
        mjModel *model_;
        mjData *data_;
        mjvCamera cam_;
        mjvOption opt_;
        mjvScene scene_;
        mjrContext context_;
        GLFWwindow *window_;
        InitConfig config_;

        bool is_paused_ = false;          // 暂停状态
        double initial_cam_[6];           // 初始相机参数存储
        mjData *initial_state_ = nullptr; // 初始状态备份

        // 鼠标状态
        bool button_left_ = false;
        bool button_middle_ = false;
        bool button_right_ = false;
        double lastx_ = 0;
        double lasty_ = 0;


    };
}
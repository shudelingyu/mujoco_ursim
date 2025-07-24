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
        MuJoCoViewer(MujocoContext &mjWrapper)
            : model_(mjWrapper.model),
              data_(mjWrapper.data),
              cam_(mjWrapper.camera),
              opt_(mjWrapper.options),
              scene_(mjWrapper.scene),
              context_(mjWrapper.render_context)
        {
            // load_xml(init,mjWrapper);
            // 初始化GLFW
            if (!glfwInit())
            {
                throw std::runtime_error("Failed to initialize GLFW");
            }

            // 创建窗口
            window_ = glfwCreateWindow(1244, 700, "MuJoCo Viewer", nullptr, nullptr);
            if (!window_)
            {
                glfwTerminate();
                throw std::runtime_error("Failed to create GLFW window");
            }
            // 设置OpenGL上下文
            glfwMakeContextCurrent(window_);
            glfwSwapInterval(1);
            mjv_defaultCamera(&cam_);
            mjv_defaultOption(&opt_);
            mjv_defaultScene(&scene_);
            mjr_defaultContext(&context_);
            mjv_makeScene(model_, &scene_, 2000);                // space for 2000 objects
            mjr_makeContext(model_, &context_, mjFONTSCALE_150); // model-specific context
            // if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
            //     glfwTerminate();
            //     throw std::runtime_error("Failed to initialize GLAD");
            // }

            // 设置用户指针以便回调访问
            glfwSetWindowUserPointer(window_, this);

            // 设置回调
            glfwSetKeyCallback(window_, &MuJoCoViewer::keyboardCallback);
            glfwSetCursorPosCallback(window_, &MuJoCoViewer::mouseMoveCallback);
            glfwSetMouseButtonCallback(window_, &MuJoCoViewer::mouseButtonCallback);
            glfwSetScrollCallback(window_, &MuJoCoViewer::scrollCallback);
            double arr_view[] = {1, 1, 2, 0.000000, 0.000000, 0.000000};
            cam_.azimuth = arr_view[0];
            cam_.elevation = arr_view[1];
            cam_.distance = arr_view[2];
            cam_.lookat[0] = arr_view[3];
            cam_.lookat[1] = arr_view[4];
            cam_.lookat[2] = arr_view[5];

            // 备份初始状态
            initial_state_ = mj_makeData(model_);
            mj_copyData(initial_state_, model_, data_);

            // 存储初始相机参数
            // double arr_view[] = {1, 1, 2, 0.0, 0.0, 0.0};
            // std::copy(arr_view, arr_view+6, initial_cam_);
            // 方法1：禁用所有接触约束,使得constraint为0
            model_->opt.disableflags |= mjDSBL_CONSTRAINT;
        }

        ~MuJoCoViewer()
        {
            if (initial_state_)
            {
                mj_deleteData(initial_state_);
            }
            mjv_freeScene(&scene_);
            mjr_freeContext(&context_);
            glfwDestroyWindow(window_);
            glfwTerminate();
        }

        void run(const double &endtime)
        {
            while (!glfwWindowShouldClose(window_))
            {
                if (!is_paused_)
                { // 只在非暂停状态下更新物理
                    mjtNum simstart = data_->time;
                    while (data_->time - simstart < 1.0 / 60.0)
                    {
                        mj_step(model_, data_);
                    }
                }
                // mjtNum simstart = data_->time;
                // while (data_->time - simstart < 1.0 / 60.0)
                // {
                //     mj_step(model_, data_);
                //     // mj_forward(m,d);
                // }
                if (endtime < 50)
                {
                    if (data_->time >= endtime)
                    {
                        // fclose(fid);
                        break;
                    }
                }

                // get framebuffer viewport
                mjrRect viewport = {0, 0, 0, 0};
                glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

                // update scene and render
                mjv_updateScene(model_, data_, &opt_, NULL, &cam_, mjCAT_ALL, &scene_);
                mjr_render(viewport, &scene_, &context_);
                // printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

                // swap OpenGL buffers (blocking call due to v-sync)
                glfwSwapBuffers(window_);

                // process pending GUI events, call GLFW callbacks
                glfwPollEvents();
            }
        }

    private:
        mjModel *model_;
        mjData *data_;
        mjvCamera cam_;
        mjvOption opt_;
        mjvScene scene_;
        mjrContext context_;
        GLFWwindow *window_;

        bool is_paused_ = false;          // 暂停状态
        double initial_cam_[6];           // 初始相机参数存储
        mjData *initial_state_ = nullptr; // 初始状态备份

        // 鼠标状态
        bool button_left_ = false;
        bool button_middle_ = false;
        bool button_right_ = false;
        double lastx_ = 0;
        double lasty_ = 0;

        // 静态回调函数
        static void keyboardCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
        {
            auto *viewer = static_cast<MuJoCoViewer *>(glfwGetWindowUserPointer(window));
            if (key == GLFW_KEY_R && action == GLFW_PRESS)
            {
                // 完全重置到初始状态
                mj_copyData(viewer->data_, viewer->model_, viewer->initial_state_);
                mj_forward(viewer->model_, viewer->data_);
                Controller::getInstance().reset();
                // 恢复相机参数
                // viewer->cam_.azimuth = viewer->initial_cam_[0];
                // viewer->cam_.elevation = viewer->initial_cam_[1];
                // viewer->cam_.distance = viewer->initial_cam_[2];
                // viewer->cam_.lookat[0] = viewer->initial_cam_[3];
                // viewer->cam_.lookat[1] = viewer->initial_cam_[4];
                // viewer->cam_.lookat[2] = viewer->initial_cam_[5];
            }

            // 添加空格键暂停控制
            if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
            {
                viewer->is_paused_ = !viewer->is_paused_;
            }
        }

        static void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods)
        {
            auto *viewer = static_cast<MuJoCoViewer *>(glfwGetWindowUserPointer(window));
            viewer->button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
            viewer->button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
            viewer->button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

            // if (viewer->button_left_) {
            //     // 获取点击位置
            //     double x, y;
            //     glfwGetCursorPos(window, &x, &y);
            //     // 转换到归一化坐标
            //     mjrRect viewport = {0, 0, 0, 0};
            //     glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
            //     // find geom and 3D click point, get corresponding body
            //     mjrRect r = pending_.select_state.rect[3];
            //     mjtNum pos[3];
            //     int selgeom, selflex, selskin;
            //     int sel = mjv_select(m, d, &opt,
            //         static_cast<mjtNum>(viewport.width) / viewport.height,
            //         (mjtNum)x/(viewport.width-1),
            //         (mjtNum)(viewport.height-1-y)/(viewport.height-1),
            //         &scn, pos, &selgeom, &selflex, &selskin);
            //     // int sel = mjv_select(m, d, &opt,
            //     //     static_cast<mjtNum>(r.width) / r.height,
            //     //     (pending_.select_state.x - r.left) / r.width,
            //     //     (pending_.select_state.y - r.bottom) / r.height,
            //     //     &scn, pos, &selgeom, &selflex, &selskin);
            //     //std::cout << "Selected: "<< sel << " wrist_3_link "<<mj_name2id(m, mjOBJ_BODY, "wrist_3_link")<< std::endl;
            //     // 如果选中机械臂末端
            //     if (sel == mj_name2id(m, mjOBJ_BODY, "ee_link")) {
            //         // 存储选中状态
            //         pert.active = 1;
            //         mju_copy3(pert.refpos, pos);
            //         ///std::cout << "Selected: "<< pos[0]<<"  "<<pos[1]<<"  "<<pos[2]<< std::endl;
            //     }
            // }
            // else {
            //     pert.active = 0;
            // }
        }

        static void mouseMoveCallback(GLFWwindow *window, double xpos, double ypos)
        {
            auto *viewer = static_cast<MuJoCoViewer *>(glfwGetWindowUserPointer(window));
            double dx = xpos - viewer->lastx_;
            double dy = ypos - viewer->lasty_;
            viewer->lastx_ = xpos;
            viewer->lasty_ = ypos;

            if (!viewer->button_left_ && !viewer->button_middle_ && !viewer->button_right_)
            {
                return;
            }

            int width, height;
            glfwGetWindowSize(window, &width, &height);
            bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                              glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

            mjtMouse action;
            if (viewer->button_right_)
            {
                action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
            }
            else if (viewer->button_left_)
            {
                action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
            }
            else
            {
                action = mjMOUSE_ZOOM;
            }

            mjv_moveCamera(viewer->model_, action, dx / height, dy / height, &viewer->scene_, &viewer->cam_);
        }

        static void scrollCallback(GLFWwindow *window, double xoffset, double yoffset)
        {
            auto *viewer = static_cast<MuJoCoViewer *>(glfwGetWindowUserPointer(window));
            mjv_moveCamera(viewer->model_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &viewer->scene_, &viewer->cam_);
        }
    };
}
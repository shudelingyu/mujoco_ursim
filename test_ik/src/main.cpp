#include <GLFW/glfw3.h>
#include "algorithm/math_util.hpp"
#include "common/controller.hpp"
#include "common/mujoco_view.hpp"

using namespace common;

// 全局单例对象
Controller& controller = Controller::getInstance();
InitConfig init_config;
MujocoContext mujoco_context;

// 初始关节位置
// const std::vector<double> initial_q = {0, 0, 0, 0, 0, 0};
const std::vector<double> initial_q = {0, -2.55, -0.785, 0, 0, 0};
// 控制器回调
void control_callback(const mjModel *model, mjData *data)
{
    // for(int i=0;i<model->nu;i++){
    //     controller.set_joint_targets(i,initial_q[i]);
    // }
    controller.compute_control(model, data);
    // if (model->nu > 0)
    // {
    //     std::cout << data->time << " "
    //               << data->ctrl[0] << " "
    //               << data->qpos[0] << " "
    //               << data->actuator_force[0] << std::endl;
    // }
}

// 初始化控制器
void initialize_controller(const mjModel *model, mjData *data)
{
    mj_forward(model, data);

    // 设置初始关节位置
    for (size_t i = 0; i < model->nu; i++)
    {
        data->qpos[i] = initial_q[i];
        data->qvel[i] = 0.0;
        data->qacc[i] = 0.0;
    }
    controller.configure_joints_mode(ControlMode::POSITION);
    // 设置PID参数（示例）

    std::vector<double> joint_kpp;
    std::vector<double> joint_kpd;
    std::vector<double> joint_kpi;
    joint_kpp.resize(model->nu, 200);
    joint_kpd.resize(model->nu, 100);
    joint_kpi.resize(model->nu, 10);
    for (int i = 0; i < model->nu; i++)
        controller.configure_joint_pid(i, joint_kpp[i], joint_kpd[i], joint_kpi[i], 0, 0); // 关节0的位置PID
}

// 主函数
int main(int argc, const char** argv) {
    
    // 初始化配置
    init_config.sim_duration = 50.0;
    
    // 加载模型
    controller.load_xml(init_config, mujoco_context, argc, argv);
    
    // 初始化控制器
    initialize_controller(mujoco_context.model, mujoco_context.data);
    
    // 创建可视化器
    MuJoCoViewer viewer(mujoco_context);
    
    // 设置控制回调
    mjcb_control = control_callback;
    
    // 运行仿真
    viewer.run(init_config.sim_duration);
    
    // 删除MuJoCo数据
    if (mujoco_context.data) {
        mj_deleteData(mujoco_context.data);
        mujoco_context.data = nullptr;
    }
    
    // 删除MuJoCo模型
    if (mujoco_context.model) {
        mj_deleteModel(mujoco_context.model);
        mujoco_context.model = nullptr;
    }
    
    // 终止GLFW
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

    return 0;
}
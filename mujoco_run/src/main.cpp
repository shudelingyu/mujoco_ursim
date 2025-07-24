#include "test_ik.h"

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
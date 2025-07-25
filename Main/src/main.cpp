#include "test_ik.h"

// 主函数
int main(int argc, const char** argv) {
    
    // 初始化控制器
    initialize_controller();
    
    // 创建可视化器
    MuJoCoViewer viewer(controller.get_mujocoContext());
    
    // 设置控制回调
    mjcb_control = control_callback;
    
    // 运行仿真
    viewer.run();
    
    // 删除MuJoCo数据
    if (controller.get_mujocoContext()->data) {
        mj_deleteData(controller.get_mujocoContext()->data);
        controller.get_mujocoContext()->data = nullptr;
    }
    
    // 删除MuJoCo模型
    if (controller.get_mujocoContext()->model) {
        mj_deleteModel(controller.get_mujocoContext()->model);
        controller.get_mujocoContext()->model = nullptr;
    }
    
    // 终止GLFW
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

    return 0;
}
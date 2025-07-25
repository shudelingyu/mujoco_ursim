#include "algorithm/math_util.hpp"
#include "common/controller.hpp"
#include "common/mujoco_view.hpp"
#include "algorithm/ur_kinematics.hpp"
#include "algorithm/Traject.hpp"


using namespace common;
using namespace algorithms;

// 全局单例对象
Controller& controller = Controller::getInstance();
ur_kinematics urKin;
std::vector<TrajectoryPoint> traject;


// 初始关节位置
// const std::vector<double> initial_q = {0, 0, 0, 0, 0, 0};
const std::vector<double> initial_q = {0, -50*DEG_TO_RAD, 50*DEG_TO_RAD, 90*DEG_TO_RAD, 90*DEG_TO_RAD, 0};
// 控制器回调
void control_callback(const mjModel *model, mjData *data)
{
    int count = data->time/0.002;
    std::vector<double> cur_joint;
     std::vector<double> tar_joint;
    cur_joint.resize(6,0.0);
    for(int i=0;i<model->njnt;i++){
        cur_joint[i] = data->qpos[i];
    }
    if(count<traject.size())
        urKin.select_slv(traject[count].point,cur_joint,tar_joint);
    else{
        tar_joint = cur_joint;
    }    

    for(int i=0;i<model->nu;i++){
        controller.set_joint_targets(i,tar_joint[i]);
    }
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
void initialize_controller()
{
    InitConfig init_config;

    QuinticTrajectory planner;
        // 初始化配置
    init_config.sim_duration = 50.0;
        // 加载模型
    controller.load_xml(init_config);
    auto context = controller.get_mujocoContext();
    auto model = context->model;
    auto data = context->data;
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
    joint_kpp.resize(model->nu, 50);
    joint_kpd.resize(model->nu, 5);
    joint_kpi.resize(model->nu, 0);
    for (int i = 0; i < model->nu; i++)
        controller.configure_joint_pid(i, joint_kpp[i], joint_kpd[i], joint_kpi[i], 0, 0); // 关节0的位置PID
    tf_t flan_pos;
        urKin.forward_kinematics(initial_q,flan_pos);
    TrajectoryPoint start;
    start.position = vec3_t(flan_pos.translation());
    start.quaternion = Eigen::Quaterniond(flan_pos.rotation());

    TrajectoryPoint end;
    end.position = start.position;
    end.position.z()-=0.1;
    end.quaternion = start.quaternion;

    planner.addSegment(start,end,3.0);

    traject = planner.generate();

    // for(auto &pt:traject){
    //     std::cout<<"pos "<<pt.position.transpose()<<" vel "<<pt.velocity.transpose()<<std::endl; 
    //     std::cout<<"tf_t"<<pt.point.matrix()<<std::endl;
    // }
}
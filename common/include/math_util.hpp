#pragma once
#include<stdbool.h> //for bool
#include <algorithm> // for std::min/max
#include <map>
#include <GLFW/glfw3.h>
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "iostream"
#include <mujoco/mujoco.h>
#include "modern_robotics.hpp"
#include "vector"
#include <Eigen/Dense>


inline constexpr double ZERO_THRESH = 0.0000000001;
typedef std::size_t Size;
typedef Eigen::VectorXd vec_t;
typedef Eigen::Vector3d vec3_t;
typedef Eigen::Affine3d tf_t;
// extern double lastx,lasty;

struct initmujoco
{
    std::string path = "/home/wtc/ur5_test/";
    std::string xmlfile = "ur5.xml";
    std::string datafile = "data.csv";
    double simend = 20;
    double r = 0.5;
    double omega = 0.5;

    double x_0, y_0;

    //related to writing data to a file
    FILE *fid;
    int loop_index = 0;
    int data_frequency = 10; 
    /* data */
};

struct MjWrapper
{
    mjModel* model = NULL;                  // MuJoCo model
    mjData* data = NULL;                   // MuJoCo data
    mjvCamera cam;                      // abstract camera
    mjvOption opt;                      // visualization options
    mjvScene scn;                       // abstract scene
    mjrContext con;                     // custom GPU context

    // holders of one step history of time and position to calculate dertivatives
    mjtNum position_history = 0;
    mjtNum previous_time = 0;
    // controller related variables
    float_t ctrl_update_freq = 100;
    mjtNum last_update = 0.0;
    mjtNum ctrl;
    mjvPerturb pert;

    /* data */
};

// extern MjWrapper _mujoco;


// 控制模式枚举
enum ControlMode {
    TORQUE_CONTROL,
    VELOCITY_CONTROL,
    POSITION_CONTROL
};

// 关节控制参数
struct JointController {
    ControlMode mode = ControlMode::POSITION_CONTROL;
    double target_pos = 0.0;    // 目标位置 (rad)
    double target_vel = 0.0;    // 目标速度 (rad/s)
    double target_torque = 0.0; // 目标力矩 (Nm)
    double vel_limit=0.33;       // 速度限幅 (rad)  
    double current_pos = 0.0;
    double current_vel = 0.0;
    
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
        double max_integral = 100.0;
        double max_output = 30.0;
        double target = 0.0;
        double current = 0.0;
    } pid;
};

struct AdmittanceParam
{
    std::vector<double> Fe = {0, 0, 0, 0, 0, 0};
    std::vector<double> M = {1, 1, 1, 1, 1, 1};
    std::vector<double> B = {10, 10, 10, 10, 10, 10};
    std::vector<double> K = {30, 30, 30, 30, 30, 30};
    std::vector<double> xe = {0, 0, 0, 0, 0, 0};
    std::vector<double> dxe = {0, 0, 0, 0, 0, 0};
    std::vector<double> ddxe = {0, 0, 0, 0, 0, 0};
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

};


// 全局控制状态
struct g_control{
  std::vector<JointController> joints; // 6个关节控制器
  int selected_joint = 0;              // 当前选中的关节 (0-5)
};

struct UrParam
{
    // DH PARAM //
    double L1{ 0 };
    double L2{ 0 };
    double W1{ 0 };
    double W2{ 0 };
    double H1{ 0 };
    double H2{ 0 };
    
    // TOOL 0, by default is 321 type
    double tool0_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    std::string tool0_pe_type;

    // BASE wrt REF, by default is 321 type 
    double base2ref_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    std::string base2ref_pe_type;

    // inertia vector, size must be 6
    std::vector<std::array<double, 10> > iv_vec;

    // mot friction vector, size must be 6
    std::vector<std::array<double, 3> > mot_frc_vec;
};



auto inline constexpr next_r(Size at, Size ld)noexcept->Size { return at + ld; }
template<typename XType, typename YType>
auto inline s_vc(Size n, const double *x, XType x_t, double *y, YType y_t) noexcept->void { for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_r(x_id, x_t), y_id = next_r(y_id, y_t))y[y_id] = x[x_id]; }
template<typename XType, typename YType>
auto inline s_vc(Size n, double alpha, const double *x, XType x_t, double *y, YType y_t) noexcept->void { for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_r(x_id, x_t), y_id = next_r(y_id, y_t))y[y_id] = alpha * x[x_id]; }
auto inline s_vc(Size n, const double *x, double *y) noexcept->void { std::copy_n(x, n, y); }
auto inline s_vc(Size n, double alpha, const double *x, double *y) noexcept->void { for (Size i(-1); ++i < n;)y[i] = alpha * x[i]; }

auto inline s_nv(Size n, double alpha, double *x) noexcept->void { for (Size i(-1); ++i < n;)x[i] *= alpha; }

inline int SIGN(const double& x)
{
	return x >= ZERO_THRESH ? 1 : (x <= -ZERO_THRESH ? -1 : 0);
}

inline bool is_zero(const double& v)
{
	return std::abs(v) < ZERO_THRESH;
}

inline bool is_same(const double& lhs, const double& rhs)
{
	return is_zero(lhs - rhs);
}

template<typename XType>
auto inline s_norm(Size n, const double *x, XType x_t) noexcept->double
{
    double norm = 0;
    for (Size i(-1), x_id{ 0 }; ++i < n; x_id = next_r(x_id, x_t))norm += x[x_id] * x[x_id];
    return std::sqrt(norm);
}
auto inline s_norm(Size n, const double *x) noexcept->double { return s_norm(n, x, 1); }


auto inline s_pp2pm(const double *pp_in, double *pm_out) noexcept->double *
{
    // 正式开始计算 //
    pm_out[3] = pp_in[0];
    pm_out[7] = pp_in[1];
    pm_out[11] = pp_in[2];

    return pm_out;
}

auto inline s_rq2rm(const double *rq_in, double *rm_out, Size rm_ld) noexcept->double *
{
    // 正式开始计算 //
    rm_out[0 * rm_ld + 0] = 1 - 2 * rq_in[1] * rq_in[1] - 2 * rq_in[2] * rq_in[2];
    rm_out[0 * rm_ld + 1] = 2 * rq_in[0] * rq_in[1] - 2 * rq_in[3] * rq_in[2];
    rm_out[0 * rm_ld + 2] = 2 * rq_in[0] * rq_in[2] + 2 * rq_in[3] * rq_in[1];

    rm_out[1 * rm_ld + 0] = 2 * rq_in[0] * rq_in[1] + 2 * rq_in[3] * rq_in[2];
    rm_out[1 * rm_ld + 1] = 1 - 2 * rq_in[0] * rq_in[0] - 2 * rq_in[2] * rq_in[2];
    rm_out[1 * rm_ld + 2] = 2 * rq_in[1] * rq_in[2] - 2 * rq_in[3] * rq_in[0];

    rm_out[2 * rm_ld + 0] = 2 * rq_in[0] * rq_in[2] - 2 * rq_in[3] * rq_in[1];
    rm_out[2 * rm_ld + 1] = 2 * rq_in[1] * rq_in[2] + 2 * rq_in[3] * rq_in[0];
    rm_out[2 * rm_ld + 2] = 1 - 2 * rq_in[0] * rq_in[0] - 2 * rq_in[1] * rq_in[1];

    return rm_out;
}

auto inline s_pq2pm(const double *pq_in, double *pm_out) noexcept->double *
{

    // 正式开始计算 //
    s_pp2pm(pq_in, pm_out);
    s_rq2rm(pq_in + 3, pm_out,4);

    pm_out[12] = 0;
    pm_out[13] = 0;
    pm_out[14] = 0;
    pm_out[15] = 1;

    return pm_out;
}

auto inline s_pm_dot_v3(double *pm,double *v3, double *v3_out)
{
    v3_out[0] = pm[0] * v3[0] + pm[1] * v3[1] + pm[2] * v3[2];
    v3_out[1] = pm[4] * v3[0] + pm[5] * v3[1] + pm[6] * v3[2];
    v3_out[2] = pm[8] * v3[0] + pm[9] * v3[1] + pm[10] * v3[2];
}

auto inline s_pm_dot_pm(const double *pm1, const double *pm2, double *pm_out) noexcept->double *
{
    pm_out[0] = pm1[0] * pm2[0] + pm1[1] * pm2[4] + pm1[2] * pm2[8];
    pm_out[1] = pm1[0] * pm2[1] + pm1[1] * pm2[5] + pm1[2] * pm2[9];
    pm_out[2] = pm1[0] * pm2[2] + pm1[1] * pm2[6] + pm1[2] * pm2[10];
    pm_out[3] = pm1[0] * pm2[3] + pm1[1] * pm2[7] + pm1[2] * pm2[11] + pm1[3];

    pm_out[4] = pm1[4] * pm2[0] + pm1[5] * pm2[4] + pm1[6] * pm2[8];
    pm_out[5] = pm1[4] * pm2[1] + pm1[5] * pm2[5] + pm1[6] * pm2[9];
    pm_out[6] = pm1[4] * pm2[2] + pm1[5] * pm2[6] + pm1[6] * pm2[10];
    pm_out[7] = pm1[4] * pm2[3] + pm1[5] * pm2[7] + pm1[6] * pm2[11] + pm1[7];

    pm_out[8] = pm1[8] * pm2[0] + pm1[9] * pm2[4] + pm1[10] * pm2[8];
    pm_out[9] = pm1[8] * pm2[1] + pm1[9] * pm2[5] + pm1[10] * pm2[9];
    pm_out[10] = pm1[8] * pm2[2] + pm1[9] * pm2[6] + pm1[10] * pm2[10];
    pm_out[11] = pm1[8] * pm2[3] + pm1[9] * pm2[7] + pm1[10] * pm2[11] + pm1[11];

    pm_out[12] = 0;
    pm_out[13] = 0;
    pm_out[14] = 0;
    pm_out[15] = 1;

    return pm_out;
}

inline std::vector<double> vecxd2stdvec(const Eigen::VectorXd& input){
    std::vector<double> ret;
    ret.clear();
    for(int i=0;i<input.size();i++){
        ret.push_back(input[i]);
    }
    return ret;
}

inline Eigen::VectorXd stdvec2vecxd(const std::vector<double>& input){
    Eigen::VectorXd ret(6);
    for(int i=0;i<input.size();i++){
        ret[i] = input[i];
    }
    return ret;
}

inline std::vector<double> double2stdvec(const double* input,const int &n){
    std::vector<double> ret;
    ret.clear();
    for(int i=0;i<n;i++){
        ret.push_back(input[i]);
    }
    return ret;
}

inline double distance(const std::vector<double> &q1,const std::vector<double> &q2){
    if(q1.size()!=q2.size())
        return INFINITY;
    double sum = 0.0;
    for(int i=0;i<q1.size();i++){
        sum+=std::abs(q1[i]-q2[i]);
    }
    return sum;
}

//求伪逆
inline Eigen::MatrixXd pinv(Eigen::MatrixXd A)
{
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
	double  pinvtoler = 1.e-8; //tolerance
	int row = A.rows();
	int col = A.cols();
	int k = std::min(row, col);
	Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col, row);
	Eigen::MatrixXd singularValues_inv = svd.singularValues();//����ֵ
	Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
	for (long i = 0; i < k; ++i) {
		if (singularValues_inv(i) > pinvtoler)
			singularValues_inv(i) = 1.0 / singularValues_inv(i);
		else singularValues_inv(i) = 0;
	}
	for (long i = 0; i < k; ++i)
	{
		singularValues_inv_mat(i, i) = singularValues_inv(i);
	}
	X = (svd.matrixV())*(singularValues_inv_mat)*(svd.matrixU().transpose());

	return X;
}
/*
	 * s_quadprog 求解二次规划问题
	 *			min 0.5 x'Gx + g0'x
	 *				s.t.
	 *                   CE x == ce0
	 *                   CI x <= ci0
	 *	参数说明：
	 *			nG : dim(x)
	 *			nCE : 等式约束个数
	 *			nCI : 不等式约束个数
	 *   注意： 1、G矩阵必须正定
	 *		  2、mem为内部只用的内存空间，大小需大于 2*nG*nG + 3*nG + 8*(nCE+nCI)
	 *		  3、函数内部会更改G、g0、CE、ce0、CI、ci0等矩阵
	 */
// inline double s_quadprog(Size nG, Size nCE, Size nCI,
//     const double *G, const double *g0,
//     const double *CE, const double *ce0,
//     const double *CI, const double *ci0,
//     double *x, double *mem);

// //力旋量
// inline Eigen::MatrixXd force_to_force(Eigen::MatrixXd &T,Eigen::MatrixXd &force_in_body)
// {
// //将末端作用力转换到世界坐标系上
// //参考现代机器人学P67
// //Js=[AdTsb]*Jb
// Eigen::MatrixXd force_in_forword;
// Eigen::MatrixXd Tba=T.inverse();
// Eigen::MatrixXd ans=mr::Adjoint(Tba);
// return force_in_forword=ans.transpose()*force_in_body;
// }
// //静力学
// //tor=J.T*F
//F=J.-T*tor





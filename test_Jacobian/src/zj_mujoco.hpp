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

using namespace mr;
typedef std::size_t Size;
template<typename XType, typename YType>
auto inline s_vc(Size n, const double *x, XType x_t, double *y, YType y_t) noexcept->void { for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_r(x_id, x_t), y_id = next_r(y_id, y_t))y[y_id] = x[x_id]; }
template<typename XType, typename YType>
auto inline s_vc(Size n, double alpha, const double *x, XType x_t, double *y, YType y_t) noexcept->void { for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_r(x_id, x_t), y_id = next_r(y_id, y_t))y[y_id] = alpha * x[x_id]; }
auto inline s_vc(Size n, const double *x, double *y) noexcept->void { std::copy_n(x, n, y); }
auto inline s_vc(Size n, double alpha, const double *x, double *y) noexcept->void { for (Size i(-1); ++i < n;)y[i] = alpha * x[i]; }


auto s_pm_dot_v3(const double *pm, const double *v3, double *v3_out) noexcept->double *
{
    v3_out[0] = pm[0] * v3[0] + pm[1] * v3[1] + pm[2] * v3[2];
    v3_out[1] = pm[4] * v3[0] + pm[5] * v3[1] + pm[6] * v3[2];
    v3_out[2] = pm[8] * v3[0] + pm[9] * v3[1] + pm[10] * v3[2];

    return v3_out;
}

//静力学
//tor=J.T*F
//F=J.-T*tor

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

// 控制模式枚举
enum ControlMode {
    TORQUE_CONTROL,
    VELOCITY_CONTROL,
    POSITION_CONTROL
};

//三次多项式插值轨迹规划
auto CubicSpline(int n, const double *x, const double *y, const double *z, double *p1, double *p2, double *p3)->void
{
    //std::cout <<" START" << std::endl;
    //std::cout << " n:"<<n << std::endl;
    for (int i = 0; i < n - 1; i++) {
        double T = x[i + 1] - x[i];
        p1[i] = z[i];
        p2[i] = (3 * y[i + 1] - 3 * y[i] - 2 * z[i] * T - z[i + 1] * T) / ::std::pow(T, 2);
        p3[i] = (2 * y[i] - 2 * y[i + 1] + z[i] * T + z[i + 1] * T) / ::std::pow(T, 3);
        //	std::cout << "p1 " << p1[i] << "  p2 " << p2[i] << "   p3 " << p3[i] << std::endl;
    }
}

auto CubicSpline_at(int n, const double *x, const double *y, const double *z, const double *p1, const double *p2, const double *p3, double xt)->double
{
    auto pos = std::upper_bound(x, x + n - 1, xt);
    std::size_t id = pos == x ? 0 : pos - x - 1;
    double w = xt - x[id];
    return ((w*p3[id] + p2[id])*w + p1[id])*w + y[id];
}


Eigen::MatrixXd ur5_FK(const mjModel* m,mjData* d){
    //采用《现代机器人学》书中旋量法对位姿进行正解

    double pm_now[16]{ 0 }, PqEnd[7] = { 0 };
    //ur5
    double L1 = 0.425;  
    double L2 = 0.39225;
    double W1 = 0.10915; 
    double W2 = 0.0823;  
    double H1 = 0.089159;
    double H2 = 0.09465; 
    //初始位姿
    Eigen::MatrixXd M(4, 4);
    M << -1, 0, 0, L1+ L2,
        0, 0, 1, W1+W2,
        0, 1, 0, H1-H2,
        0, 0, 0, 1;
    Eigen::MatrixXd Slist(6, 6);
    //s矩阵
    //列排列，非行排列
    Slist << 0, 0,    0,    0,		0,		0,
                0, 1,    1,    1,		0,		1,
                1, 0,    0,	0,		-1,		0,
                0, -H1, -H1,  -H1,		-W1,	H2 - H1,
                0, 0,    0,	0,		L1 + L2,0,
                0, 0,    L1,	L1 + L2, 0,		 L1 + L2;
    //std::cout << "Slist :" << Slist << std::endl;

    Eigen::VectorXd thetaList(6);
    thetaList << d->qpos[0], d->qpos[1], d->qpos[2],
    d->qpos[3], d->qpos[4], d->qpos[5];
    Eigen::MatrixXd FKCal = mr::FKinSpace(M, Slist, thetaList);

    // std::cout << "FKCals :" << std::endl;
    // std::cout << FKCal << std::endl;
    // std::cout  << std::endl; 
    
    // std::cout<<"sensor: "<<d->sensordata[12]<<"  "<<d->sensordata[13]<<"  "<<d->sensordata[14]<<std::endl;
    return FKCal;
}


// Eigen::MatrixXd ur5_FK(const mjModel* m,mjData* d){
//     //采用《现代机器人学》书中旋量法对位姿进行正解

//     double pm_now[16]{ 0 }, PqEnd[7] = { 0 };
//     //ur5
//     double L1 = 0.425;  
//     double L2 = 0.39225;
//     double W1 = 0.10915; 
//     double W2 = 0.0823;  
//     double H1 = 0.089159;
//     double H2 = 0.09465; 
//     //初始位姿
//     Eigen::MatrixXd M(4, 4);
//     M << -1, 0, 0, L1+ L2,
//         0, 0, 1, W1+W2,
//         0, 1, 0, H1-H2,
//         0, 0, 0, 1;
//     Eigen::MatrixXd Slist(6, 6);
//     //s矩阵
//     //列排列，非行排列
//     Slist << 0, 0,    0,    0,		0,		0,
//                 0, 1,    1,    1,		0,		1,
//                 1, 0,    0,	0,		-1,		0,
//                 0, -H1, -H1,  -H1,		-W1,	H2 - H1,
//                 0, 0,    0,	0,		L1 + L2,0,
//                 0, 0,    L1,	L1 + L2, 0,		 L1 + L2;
//     //std::cout << "Slist :" << Slist << std::endl;

//     Eigen::VectorXd thetaList(6);
//     thetaList << d->qpos[0], d->qpos[1], d->qpos[2],
//     d->qpos[3], d->qpos[4], d->qpos[5];
//     Eigen::MatrixXd FKCal = mr::FKinSpace(M, Slist, thetaList);

//     std::cout << "FKCals :" << std::endl;
//     std::cout << FKCal << std::endl;
//     std::cout  << std::endl; 
    
//     std::cout<<"sensor: "<<d->sensordata[12]<<"  "<<d->sensordata[13]<<"  "<<d->sensordata[14]<<std::endl;
//     return FKCal;
// }


Eigen::MatrixXd ur5_FK(const mjModel* m,mjData* d, double *qpos){
    //采用《现代机器人学》书中旋量法对位姿进行正解

    double pm_now[16]{ 0 }, PqEnd[7] = { 0 };
    //ur5
    double L1 = 0.425;  
    double L2 = 0.39225;
    double W1 = 0.10915; 
    double W2 = 0.0823;  
    double H1 = 0.089159;
    double H2 = 0.09465; 
    //初始位姿
    Eigen::MatrixXd M(4, 4);
    M << -1, 0, 0, L1+ L2,
        0, 0, 1, W1+W2,
        0, 1, 0, H1-H2,
        0, 0, 0, 1;
    Eigen::MatrixXd Slist(6, 6);
    //s矩阵
    //列排列，非行排列
    Slist << 0, 0,    0,    0,		0,		0,
                0, 1,    1,    1,		0,		1,
                1, 0,    0,	0,		-1,		0,
                0, -H1, -H1,  -H1,		-W1,	H2 - H1,
                0, 0,    0,	0,		L1 + L2,0,
                0, 0,    L1,	L1 + L2, 0,		 L1 + L2;
    //std::cout << "Slist :" << Slist << std::endl;

    Eigen::VectorXd thetaList(6);
    thetaList << qpos[0], qpos[1], qpos[2],
    qpos[3], qpos[4], qpos[5];
    Eigen::MatrixXd FKCal = mr::FKinSpace(M, Slist, thetaList);

    // std::cout << "FKCals :" << std::endl;
    // std::cout << FKCal << std::endl;
    // std::cout  << std::endl; 
    
    // std::cout<<"sensor: "<<d->sensordata[12]<<"  "<<d->sensordata[13]<<"  "<<d->sensordata[14]<<std::endl;
    return FKCal;
}


Eigen::MatrixXd ur5_JacobianSpace(const mjModel* m,mjData* d){
    //ur5
    double L1 = 0.425;  
    double L2 = 0.39225;
    double W1 = 0.10915; 
    double W2 = 0.0823;  
    double H1 = 0.089159;
    double H2 = 0.09465; 
    Eigen::MatrixXd Slist(6, 6);
    //s矩阵
    //列排列，非行排列
    Slist << 0, 0,    0,    0,		0,		0,
                0, 1,    1,    1,		0,		1,
                1, 0,    0,	0,		-1,		0,
                0, -H1, -H1,  -H1,		-W1,	H2 - H1,
                0, 0,    0,	0,		L1 + L2,0,
                0, 0,    L1,	L1 + L2, 0,		 L1 + L2;

    Eigen::VectorXd thetaList(6);
    thetaList << d->qpos[0], d->qpos[1], d->qpos[2],
    d->qpos[3], d->qpos[4], d->qpos[5];

    std::cout << "thetaList :\n" << thetaList<< std::endl;
    Eigen::MatrixXd result = mr::JacobianSpace(Slist, thetaList);
    return result;
}


Eigen::MatrixXd ur5_JacobianBody(const mjModel* m,mjData* d){
    //ur5
    double L1 = 0.425;  
    double L2 = 0.39225;
    double W1 = 0.10915; 
    double W2 = 0.0823;  
    double H1 = 0.089159;
    double H2 = 0.09465; 
    Eigen::MatrixXd Slist(6, 6);
    //s矩阵
    //列排列，非行排列
    Slist << 0, 0,    0,    0,		0,		0,
                0, 1,    1,    1,		0,		1,
                1, 0,    0,	0,		-1,		0,
                0, -H1, -H1,  -H1,		-W1,	H2 - H1,
                0, 0,    0,	0,		L1 + L2,0,
                0, 0,    L1,	L1 + L2, 0,		 L1 + L2;

    Eigen::VectorXd thetaList(6);
    thetaList << d->qpos[0], d->qpos[1], d->qpos[2],
    d->qpos[3], d->qpos[4], d->qpos[5];
    Eigen::MatrixXd result = mr::JacobianBody(Slist, thetaList);
    return result;
}




void ur5_IK1(const mjModel* m,mjData* d,Eigen::MatrixXd T){

    double L1 = 0.425;  
    double L2 = 0.39225;
    double W1 = 0.10915; 
    double W2 = 0.0823;  
    double H1 = 0.089159;
    double H2 = 0.09465; 
    //初始位姿
    Eigen::MatrixXd M(4, 4);
    M << -1, 0, 0, L1+ L2,
        0, 0, 1, W1+W2,
        0, 1, 0, H1-H2,
        0, 0, 0, 1;
    Eigen::MatrixXd SlistT(6, 6);
    //s矩阵
    //列排列，非行排列
    SlistT << 0, 0,    0,    0,		0,		0,
                0, 1,    1,    1,		0,		1,
                1, 0,    0,	0,		-1,		0,
                0, -H1, -H1,  -H1,		-W1,	H2 - H1,
                0, 0,    0,	0,		L1 + L2,0,
                0, 0,    L1,	L1 + L2, 0,		 L1 + L2;
    Eigen::MatrixXd Slist = SlistT.transpose();

    Eigen::VectorXd thetaList(6);
    thetaList << d->qpos[0], d->qpos[1], d->qpos[2],
    d->qpos[3], d->qpos[4], d->qpos[5];

    double eomg = 0.1;
    double ev = 0.004;
    bool b_result = true;
    // Eigen::VectorXd theta_result(3);
    // theta_result << 1.57073783, 2.99966384, 3.1415342;
    bool iRet = mr::IKinSpace(Slist, M, T, thetaList, eomg, ev);
    if (iRet)
    {
        std::cout << "IKinSpace success" << std::endl;
        std::cout << "thetaList :" << thetaList << std::endl;
        for (size_t i = 0; i < m->nu; i++)
        {
            std::cout << "  " << d->qpos[i];
        }
        std::cout <<std::endl;
        
    }
    else
    {
        std::cout << "IKinSpace failed" << std::endl;
    }
    
}


    
    
auto URInverseKinematic(const double *ee_pm, int which_root, double *input)->bool
{
    UrParam param;
    param.L1 = 0.425;  
    param.L2 = 0.39225;
    param.W1 = 0.10915; 
    param.W2 = 0.0823;  
    param.H1 = 0.089159;
    param.H2 = 0.09465; 


    double a[6] = { 0, -param.L1, -param.L2, 0, 0, 0 };
    double d[6] = { param.H1, 0, 0, param.W1, param.H2, param.W2 };
    double nx = ee_pm[0], ny = ee_pm[4], nz = ee_pm[8];
    double ox = ee_pm[1], oy = ee_pm[5], oz = ee_pm[9];
    double ax = ee_pm[2], ay = ee_pm[6], az = ee_pm[10];
    double px = ee_pm[3], py = ee_pm[7], pz = ee_pm[11];
    double q[6]{ 0 };
    //求解关节角1//
    double m = d[5]*ay - py,  n = ax * d[5] - px;
    if ((m*m + n * n - d[3] * d[3]) < 0) return false;
    if (which_root & 0x04)
    {
        q[0] = atan2(m, n) - atan2(d[3], sqrt(m*m + n*n - d[3]*d[3]));
    }
    else
    {
        q[0] = atan2(m, n) - atan2(d[3], -sqrt(m*m + n * n - d[3] * d[3]));
    }
    
    //求解关节角5//
    if ((ax*sin(q[0]) - ay * cos(q[0])) > 1) return false;
    if (which_root & 0x02)
    {
        q[4] = acos(ax*sin(q[0]) - ay * cos(q[0]));
    }
    else
    {
        q[4] = -acos(ax*sin(q[0]) - ay * cos(q[0]));
    }
    
    //求解关节角6//
    double mm = nx * sin(q[0]) - ny * cos(q[0]);
    double nn = ox * sin(q[0]) - oy * cos(q[0]);
    q[5] = atan2(mm, nn) - atan2(sin(q[4]), 0); 
    //求解关节角3//
    double mmm = d[4]*(sin(q[5])*(nx*cos(q[0]) + ny * sin(q[0])) + cos(q[5])*(ox*cos(q[0]) + oy * sin(q[0])))
        - d[5]*(ax*cos(q[0]) + ay * sin(q[0])) + px * cos(q[0]) + py * sin(q[0]);
    double nnn = pz - d[0] - az * d[5] + d[4]*(oz*cos(q[5]) + nz * sin(q[5]));
    if ((mmm*mmm + nnn * nnn) > (a[1]*a[1]+a[2]*a[2]+2*a[1]*a[2])) return false;
    if (which_root & 0x01)
    {
    q[2] = acos((mmm*mmm + nnn*nnn - a[1]*a[1] - a[2]*a[2]) / (2 * a[1]*a[2]));
    }
    else
    {
        q[2] = -acos((mmm*mmm + nnn * nnn - a[1] * a[1] - a[2] * a[2]) / (2 * a[1] * a[2]));
    }
    
    //求解关节角2//

    double s2 = ((a[2]*cos(q[2]) + a[1])*nnn - a[2]*sin(q[2])*mmm)/
        (a[1]*a[1] + a[2]*a[2] + 2 * a[1]*a[2]*cos(q[2]));
    double c2 = (mmm + a[2]*sin(q[2])*s2)/ (a[2]*cos(q[2]) + a[1]);
    q[1] = atan2(s2, c2);

    //求解关节角4//
    q[3] = atan2(-sin(q[5])*(nx*cos(q[0]) + ny * sin(q[0])) - cos(q[5])*
        (ox*cos(q[0]) + oy * sin(q[0])), oz*cos(q[5]) + nz * sin(q[5])) - q[1] - q[2];

    // 添加所有的偏移 //
    for (int i = 0; i < 6; ++i)
    {
        while (q[i] > M_PI) q[i] -= 2 * M_PI;
        while (q[i] < -M_PI) q[i] += 2 * M_PI;
    }

    // 将q copy到input中
    s_vc(6, q, input);
    return true;
}

auto InverseKinematic(const double *ee_pm,mjData* d){
    int solution_num = 0;
    double diff_q[8][6];
    double diff_norm[8];
    // double ans_pos[6];
    Eigen::VectorXd thetaList(6);
    for (int i = 0; i < 8; ++i)
    {
        if (URInverseKinematic(ee_pm, i, diff_q[solution_num]))
        {
            diff_norm[solution_num] = 0;
            for (int j = 0; j < 6; ++j)
            {
                diff_q[solution_num][j] -= d->qpos[j];
    
                while (diff_q[solution_num][j] > M_PI) diff_q[solution_num][j] -= 2 * M_PI;
                while (diff_q[solution_num][j] < -M_PI)diff_q[solution_num][j] += 2 * M_PI;
    
                diff_norm[solution_num] += std::abs(diff_q[solution_num][j]);
            }
    
            ++solution_num;
        }
    }
    
    //if (solution_num == 0) return -1;
    auto real_solution = std::min_element(diff_norm, diff_norm + solution_num) - diff_norm;
    //选解策略（距离最近）
    //std::cout << "ik_pos :" << std::endl;
    for (uint j = 0; j < 6; ++j)
    {
        //std::cout <<" "<< d->qpos[j];
        thetaList[j] = d->qpos[j] + diff_q[real_solution][j];
        while (thetaList[j] > 2 * M_PI) thetaList[j] -= 2 * M_PI;
        while (thetaList[j] < -2 * M_PI)thetaList[j] += 2 * M_PI;
    }
    // std::cout<<std::endl;
    // std::cout << "ik_solution :" << std::endl;
    // //std::cout<< ans_pos[0] <<"  "<< ans_pos[1] <<"  "<< ans_pos[2] <<"  "<< ans_pos[3] <<"  "<< ans_pos[4] <<"  "<< ans_pos[5] <<"  "<< std::endl;
    // std::cout<< thetaList[0] <<"  "<< thetaList[1] <<"  "<< thetaList[2] <<"  "<< thetaList[3] <<"  "<< thetaList[4] <<"  "<< thetaList[5] <<"  "<< std::endl;
    // std::cout<< d->qpos[0] <<"  "<< d->qpos[1] <<"  "<< d->qpos[2] <<"  "<< d->qpos[3] <<"  "<< d->qpos[4] <<"  "<< d->qpos[5] <<"  "<< std::endl;

    // s_vc(6, diff_q[real_solution], d->qpos);
    // return real_solution;
    return thetaList;
}

//求伪逆
Eigen::MatrixXd pinv(Eigen::MatrixXd A)
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


// 将关节速度转换为笛卡尔速度（线速度 + 角速度）
Eigen::VectorXd jointVelToCartesianVel(
    mjModel* m, 
    mjData* d, 
    const char* site_name
) {
    // 获取site的ID
    int site_id = mj_name2id(m, mjOBJ_SITE, site_name);
    if (site_id == -1) {
        throw std::runtime_error("Site未找到");
    }

    // 获取关节速度（qvel）
    Eigen::VectorXd qdot = Eigen::Map<Eigen::VectorXd>(
        d->qvel, m->nv
    );

    // 计算完整的雅可比矩阵（6x nv：3行线速度，3行角速度）
    Eigen::MatrixXd jac(6, m->nv);
    mj_jac(m, d, jac.block(0,0,3,m->nv).data(), // 位置雅可比（前3行）
           jac.block(3,0,3,m->nv).data(),      // 旋转雅可比（后3行）
           NULL,site_id);

    // 计算笛卡尔速度：v_cartesian = J * q_dot
    Eigen::VectorXd v_cartesian = jac * qdot;

    return v_cartesian; // 返回6维向量 [线速度; 角速度]
}









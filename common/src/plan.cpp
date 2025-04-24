#include "plan.hpp"


Planning::Planning(){

}

Planning::~Planning(){

}

//三次多项式插值轨迹规划
void Planning::CubicSpline(int n, const double *x, const double *y, const double *z, double *p1, double *p2, double *p3)
{
    //std::cout <<" START" << std::endl;
    //std::cout << " n:"<<n << std::endl;
    for (int i = 0; i < n - 1; i++) {
        double T = x[i + 1] - x[i];
        p1[i] = z[i];
        p2[i] = (3 * y[i + 1] - 3 * y[i] - 2 * z[i] * T - z[i + 1] * T) / ::std::pow(T, 2);
        p3[i] = (2 * y[i] - 2 * y[i + 1] + z[i] * T + z[i + 1] * T) / ::std::pow(T, 3);
        std::cout << "p1 " << p1[i] << "  p2 " << p2[i] << "   p3 " << p3[i] << std::endl;
    }
}

double Planning::CubicSpline_at(int n, const double *x, const double *y, const double *z, const double *p1, const double *p2, const double *p3, double xt)
{
    auto pos = std::upper_bound(x, x + n - 1, xt);
    std::size_t id = pos == x ? 0 : pos - x - 1;
    double w = xt - x[id];
    return ((w*p3[id] + p2[id])*w + p1[id])*w + y[id];
}

// 将关节速度转换为笛卡尔速度（线速度 + 角速度）
Eigen::VectorXd Planning::jointVelToCartesianVel(
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
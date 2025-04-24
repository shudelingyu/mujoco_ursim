#pragma once
#include "math_util.hpp"
#include <mujoco/mujoco.h>

class Planning
{
public:
    Planning();
    ~Planning();
public:
    void CubicSpline(int n, const double *x, const double *y, const double *z, double *p1, double *p2, double *p3);
    double CubicSpline_at(int n, const double *x, const double *y, const double *z, const double *p1, const double *p2, const double *p3, double xt);
    Eigen::VectorXd jointVelToCartesianVel(mjModel* m, mjData* d, const char* site_name);
};
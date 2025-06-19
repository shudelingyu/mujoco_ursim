// #include<stdbool.h> //for bool
// #include <algorithm> // for std::min/max
// #include <mujoco/mujoco.h>
// #include <map>
// #include "stdio.h"
// #include "stdlib.h"
// #include "string.h"
// #include "iostream"
// #include "vector"
#pragma once
#include "modern_robotics.hpp"
#include "math_util.hpp"

using namespace mr;


class ur_frame
{
public:
static void init(const UrParam &ur_dh);
static ur_frame*instance();
~ur_frame();

public:
    Eigen::MatrixXd forward_kinematics(const mjModel* m,mjData* d);
    Eigen::MatrixXd forward_kinematics(const mjModel* m,mjData* d,double *qpos);
    Eigen::MatrixXd jacobian_space(const mjModel* m,mjData* d);
    Eigen::MatrixXd jacobian_body(const mjModel* m,mjData* d);
    Eigen::VectorXd inverse_kinematics(const mjModel* m,mjData* d,Eigen::MatrixXd T);
    bool inverse_kinematics(const tf_t flange_pose, const vec3_t &sign_sln, const double &q6_des, std::vector<double> &sln);
    bool inverse_kinematics(const tf_t flange_pose, std::vector<std::vector<double>> &slns);
    bool inverse_kinematics(const double *ee_pm, int which_root, double *input);
    bool select_sln(const tf_t ee_pm,mjData* d,std::vector<double> &sln);
    Eigen::VectorXd select_sln(const double *ee_pm,mjData* d);

private:
    bool __is_within_limit(const std::vector<double>& joint);
    bool __convert_sln(const std::vector<double> &joint,std::vector<double> &sln);
    bool __sort_slns(const std::vector<double> &joint,std::vector<std::vector<double>>  &slns);

private:
    static ur_frame* singleton;
    ur_frame(const UrParam &ur_dh);
    UrParam param;

};
















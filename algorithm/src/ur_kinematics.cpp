#include "ur_kinematics.hpp"
#include <map>
namespace algorithms
{
    ur_kinematics::ur_kinematics()
    {
        dh_params_ = {
            {0.1625, 0, M_PI / 2, 0},
            {0, -0.425, 0, 0},
            {0, -0.3922, 0, 0},
            {0.1333, 0, M_PI / 2, 0},
            {0.0997, 0, -M_PI / 2, 0},
            {0.0996, 0, 0, 0}};
    }

    void ur_kinematics::forward_kinematics(const std::vector<double> &joints, tf_t &arm2flan)
    {
        if (joints.size() != 6)
        {
        }
        // 从基座到末端的累积变换
        arm2flan = tf_t::Identity();

        // 逐个关节应用变换
        for (int i = 0; i < 6; i++)
        {
            arm2flan = arm2flan * __transform(dh_params_[i], joints[i]);
        }
    }

    bool ur_kinematics::inverse_kinematics(const tf_t &flan_pose, std::vector<std::vector<double>> &slns)
    {
        std::vector<vec3_t> sign_slns = {{1, 1, 1}, {1, 1, -1}, {1, -1, 1}, {1, -1, -1}, {-1, -1, -1}, {-1, -1, 1}, {-1, 1, -1}, {-1, 1, 1}};
        slns.clear();
        std::vector<double> sln;
        for (int i = 0; i < sign_slns.size(); i++)
        {
            if (__inverse_kinematics(flan_pose, sign_slns[i], 0, sln) && __is_within_limit(sln))
            {
                slns.push_back(sln);
            }
        }
        return !slns.empty();
    }

    bool ur_kinematics::select_slv(const tf_t &flan_pose, const std::vector<double> &cur_joints, std::vector<double> &target_joints)
    {
        std::vector<std::vector<double>> slns;
        slns.clear();
        target_joints.clear();
        if (this->inverse_kinematics(flan_pose, slns))
        {
            if (this->__sort_slns_jointlike(cur_joints, slns))
            {
                target_joints = slns[0];
                return true;
            }
            return false;
        }
        return false;
    }

    tf_t ur_kinematics::__transform(const DHParameters &dh, double joint_angle)
    {
        tf_t T = tf_t::Identity();
        double theta = dh.theta + joint_angle;

        // DH变换矩阵：RotZ * TransZ * TransX * RotX
        T.matrix() << cos(theta), -sin(theta) * cos(dh.alpha), sin(theta) * sin(dh.alpha), dh.a * cos(theta),
            sin(theta), cos(theta) * cos(dh.alpha), -cos(theta) * sin(dh.alpha), dh.a * sin(theta),
            0, sin(dh.alpha), cos(dh.alpha), dh.d,
            0, 0, 0, 1;

        return T;
    }

    bool ur_kinematics::__convert_sln_jointlike(const std::vector<double> &joints, std::vector<double> &sln)
    {
        if (sln.size() != 6 || joints.size() != 6)
        {
            return false;
        }
        for (int i = 0; i < sln.size(); i++)
        {
            double tmp = sln[i] < 0 ? sln[i] + 2 * M_PI : sln[i] - 2 * M_PI;
            if (tmp < 2 * M_PI && tmp > -2 * M_PI && std::abs(tmp - joints[i]) < std::abs(sln[i] - joints[i]))
            {
                sln[i] = tmp;
            }
        }
        return true;
    }

    bool ur_kinematics::__sort_slns_jointlike(const std::vector<double> &joints, std::vector<std::vector<double>> &slns)
    {
        std::multimap<double, std::vector<double>> cost_slns;
        std::vector<std::vector<double>> tmp_slns = slns;
        for (std::vector<double> &sln : tmp_slns)
        {
            if (!__convert_sln_jointlike(joints, sln))
            {
                return false;
            }
            cost_slns.insert(std::make_pair(distance(joints, sln), sln));
        }
        slns.clear();
        for (const std::pair<double, std::vector<double>> &it : cost_slns)
        {
            slns.push_back(it.second);
        }
        return true;
    }

    bool ur_kinematics::__inverse_kinematics(const tf_t &flange_pose, const vec3_t &sign_sln, const double &q6_des, std::vector<double> &sln)
    {
        sln.resize(6);
        //
        double d6 = dh_params_[5].d;
        double d5 = dh_params_[4].d;
        double d4 = dh_params_[3].d;
        double d1 = dh_params_[0].d;
        double a3 = dh_params_[2].a;
        double a2 = dh_params_[1].a;

        // 计算q1-机座
        double A = d6 * flange_pose(1, 2) - flange_pose(1, 3);
        double B = d6 * flange_pose(0, 2) - flange_pose(0, 3);
        double R = A * A + B * B;
        if (is_near_zero(A))
        {
            double div;
            if (are_values_close(fabs(d4), fabs(B)))
                div = -sign(d4) * sign(B);
            else
                div = -d4 / B;
            double arcsin = asin(div);
            if (is_near_zero(arcsin))
                arcsin = 0.0;
            sln[0] = arcsin;
            if (sign_sln.x() < 0)
                sln[0] = M_PI - sln[0];
        }
        else if (is_near_zero(B))
        {
            double div;
            if (are_values_close(fabs(d4), fabs(A)))
                div = sign(d4) * sign(A);
            else
                div = d4 / A;
            double arccos = acos(div);
            sln[0] = arccos;
            if (sign_sln.x() < 0)
                sln[0] = 2 * M_PI - arccos;
        }
        else if (d4 * d4 > R)
        {
            return false;
        }
        else
        {
            double arccos = acos(d4 / sqrt(R));
            double arctan = atan2(-B, A);
            double pn = sign_sln.x() * arccos + arctan;
            if (is_near_zero(pn))
                pn = 0.0;
            sln[0] = pn;
        }
        // 计算q5-腕2
        double numer = (flange_pose(0, 3) * sin(sln[0]) - flange_pose(1, 3) * cos(sln[0]) - d4);
        double div;
        if (are_values_close(fabs(numer), fabs(d6)))
            div = sign(numer) * sign(d6);
        else
            div = numer / d6;
        double arccos = acos(div);
        if (sign_sln.z() > 0)
            sln[4] = arccos;
        else
            sln[4] = -arccos;
        // 计算q6-腕3
        double c1 = cos(sln[0]), s1 = sin(sln[0]);
        double c5 = cos(sln[4]), s5 = sin(sln[4]);
        if (is_near_zero(s5))
        {
            sln[5] = q6_des;
        }
        else
        {
            sln[5] = atan2(sign(s5) * -(flange_pose(0, 1) * s1 - flange_pose(1, 1) * c1), sign(s5) * (flange_pose(0, 0) * s1 - flange_pose(1, 0) * c1));
            if (fabs(sln[5]) < ZERO_THRESH)
                sln[5] = 0.0;
        }
        // 计算q2,q3,q4
        double c6 = cos(sln[5]), s6 = sin(sln[5]);
        double x04x = -s5 * (flange_pose(0, 2) * c1 + flange_pose(1, 2) * s1) -
                      c5 * (s6 * (flange_pose(0, 1) * c1 + flange_pose(1, 1) * s1) - c6 * (flange_pose(0, 0) * c1 + flange_pose(1, 0) * s1));
        double x04y = c5 * (flange_pose(2, 0) * c6 - flange_pose(2, 1) * s6) - flange_pose(2, 2) * s5;
        double p13x = d5 * (s6 * (flange_pose(0, 0) * c1 + flange_pose(1, 0) * s1) + c6 * (flange_pose(0, 1) * c1 + flange_pose(1, 1) * s1)) -
                      d6 * (flange_pose(0, 2) * c1 + flange_pose(1, 2) * s1) + flange_pose(0, 3) * c1 + flange_pose(1, 3) * s1;
        double p13y = flange_pose(2, 3) - d1 - d6 * flange_pose(2, 2) + d5 * (flange_pose(2, 1) * c6 + flange_pose(2, 0) * s6);

        double c3 = (p13x * p13x + p13y * p13y - a2 * a2 - a3 * a3) / (2.0 * a2 * a3);
        if (are_values_close(fabs(c3), 1.0))
            c3 = sign(c3);
        else if (fabs(c3) > 1.0)
        {
            return false;
        }
        double arccos3 = acos(c3);
        sln[2] = sign_sln.y() > 0 ? arccos3 : -arccos3;
        double denom = a2 * a2 + a3 * a3 + 2 * a2 * a3 * c3;
        double s3 = sin(arccos3);
        double AA = (a2 + a3 * c3), BB = a3 * s3;
        sln[1] = atan2((AA * p13y - sign_sln.y() * BB * p13x) / denom, (AA * p13x + sign_sln.y() * BB * p13y) / denom);
        double c23_0 = cos(sln[1] + sln[2]);
        double s23_0 = sin(sln[1] + sln[2]);
        sln[3] = atan2(c23_0 * x04y - s23_0 * x04x, x04x * c23_0 + x04y * s23_0);

        ////////////////////////////////////////////////////////////////////////////////
        if (is_near_zero(sln[1]))
            sln[1] = 0.0;
        if (is_near_zero(sln[3]))
            sln[3] = 0.0;
        return true;
    }

    bool ur_kinematics::__is_within_limit(const std::vector<double> &joints)
    {
        if (joints.size() != 6)
        {
            return false;
        }
        for (int i = 0; i < joints.size(); i++)
        {
            if ((joints[i] > 2 * M_PI) || (joints[i] < -2 * M_PI))
            {
                return false;
            }
        }
        return true;
    }
}
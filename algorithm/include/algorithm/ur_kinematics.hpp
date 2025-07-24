#ifndef UR_KINEMATICS_H
#define UR_KINEMATICS_H
#include <Eigen/Dense>
#include "math_util.hpp"

namespace algorithms
{

    class ur_kinematics
    {
    public:
        ur_kinematics();

    public:
        void forward_kinematics(const std::vector<double> &joints, tf_t &arm2flan);
        bool inverse_kinematics(const tf_t &flan_pose, std::vector<std::vector<double>> &slns);
        bool select_slv(const tf_t &flan_pose, const std::vector<double> &cur_joints, std::vector<double> &target_joints);

    private:
        tf_t __transform(const DHParameters &dh, double joint_angle);
        bool __inverse_kinematics(const tf_t &flange_pose, const vec3_t &sign_sln, const double &q6_des, std::vector<double> &sln);
        bool __is_within_limit(const std::vector<double> &joints);
        bool __convert_sln_jointlike(const std::vector<double> &joints, std::vector<double> &sln);
        bool __sort_slns_jointlike(const std::vector<double> &joints, std::vector<std::vector<double>> &slns);

    private:
        std::vector<DHParameters> dh_params_;
    };
}
#endif // UR_KINEMATICS_H

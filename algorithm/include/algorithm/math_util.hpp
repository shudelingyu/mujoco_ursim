#pragma once

#include <Eigen/Dense>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <cstring>
#include <iostream>
#include <map>
#include <vector>
#include "modern_robotics.hpp"
namespace algorithms
{
    // 常量定义
    constexpr double ZERO_THRESH = 1e-10;
    constexpr double PI = M_PI;
    constexpr double DEG_TO_RAD = PI / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / PI;

    // 类型别名
    using Size = std::size_t;
    using MatrixXd = Eigen::MatrixXd;
    using Matrix4d = Eigen::Matrix4d;
    using Matrix3d = Eigen::Matrix3d;
    typedef Eigen::Affine3d tf_t;
    typedef Eigen::VectorXd vec_t;
    typedef Eigen::Vector3d vec3_t;

// DH参数结构体
struct DHParameters {
    double d;      // 沿Z轴的平移
    double a;      // 沿X轴的平移
    double alpha;  // 绕X轴的旋转
    double theta;  // 绕Z轴的旋转
};


    // 导纳控制参数
    struct AdmittanceParams
    {
        std::vector<double> external_force = {0, 0, 0, 0, 0, 0};
        std::vector<double> mass_matrix = {1, 1, 1, 1, 1, 1};
        std::vector<double> damping_matrix = {10, 10, 10, 10, 10, 10};
        std::vector<double> stiffness_matrix = {30, 30, 30, 30, 30, 30};
        std::vector<double> error = {0, 0, 0, 0, 0, 0};
        std::vector<double> error_derivative = {0, 0, 0, 0, 0, 0};
        std::vector<double> error_second_derivative = {0, 0, 0, 0, 0, 0};
    };

    // 伺服轨迹参数
    struct TrajectoryParams
    {
        std::vector<double> joint_positions;
        std::vector<double> joint_velocities;
        std::vector<double> time_points;
        double scaling_factor;
        std::vector<std::vector<double>> joint_position_matrix;
        std::vector<std::vector<double>> joint_velocity_matrix;
    };

    // UR机器人参数
    struct UR5Parameters
    {
        // DH 参数
        struct DHParams
        {
            double L1 = 0;
            double L2 = 0;
            double W1 = 0;
            double W2 = 0;
            double H1 = 0;
            double H2 = 0;
        } dh_params;

        // 工具坐标系
        double tool_pose[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::string tool_pose_type = "321";

        // 基坐标系
        double base_pose[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::string base_pose_type = "321";

        // 惯量参数
        std::vector<std::array<double, 10>> inertia_vectors;

        // 摩擦力参数
        std::vector<std::array<double, 3>> friction_params;
    };

    // ======================= 数学工具函数 =======================

    // 符号函数
    constexpr int sign(double x)
    {
        return (x > ZERO_THRESH) ? 1 : ((x < -ZERO_THRESH) ? -1 : 0);
    }

    // 判断是否接近零
    constexpr bool is_near_zero(double value)
    {
        return std::abs(value) < ZERO_THRESH;
    }

    // 判断两个值是否接近
    constexpr bool are_values_close(double a, double b)
    {
        return is_near_zero(a - b);
    }

    // 计算向量范数
    template <typename VectorType>
    double vector_norm(const VectorType &vec)
    {
        return vec.norm();
    }

    // 位置向量转换为齐次变换矩阵
    inline void position_to_matrix(const double *position, Matrix4d &matrix)
    {
        matrix.setIdentity();
        matrix.block<3, 1>(0, 3) = Eigen::Map<const vec3_t>(position);
    }

    // 四元数转换为旋转矩阵
    inline void quaternion_to_matrix(const double *quat, Matrix3d &rotation)
    {
        const double w = quat[3], x = quat[0], y = quat[1], z = quat[2];
        rotation << 1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w,
            2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w,
            2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y;
    }

    // 位置+四元数转换为齐次变换矩阵
    inline void pose_to_matrix(const double *position_quat, Matrix4d &transform)
    {
        transform.setIdentity();
        transform.block<3, 1>(0, 3) = Eigen::Map<const vec3_t>(position_quat);

        Matrix3d rotation;
        quaternion_to_matrix(position_quat + 3, rotation);
        transform.block<3, 3>(0, 0) = rotation;
    }

    // 矩阵乘法：4x4 * 4x4
    inline void matrix_multiply(const Matrix4d &a, const Matrix4d &b, Matrix4d &result)
    {
        result = a * b;
    }

    // 矩阵乘法：4x4 * 3D向量
    inline void matrix_vector_multiply(const Matrix4d &matrix, const vec3_t &vector, vec3_t &result)
    {
        result = matrix.block<3, 3>(0, 0) * vector + matrix.block<3, 1>(0, 3);
    }

    // 矩阵乘法：4x4 * 3D向量（重载）
    inline vec3_t matrix_vector_multiply(const Matrix4d &matrix, const vec3_t &vector)
    {
        return matrix.block<3, 3>(0, 0) * vector + matrix.block<3, 1>(0, 3);
    }

    // Eigen向量转std::vector
    inline std::vector<double> eigen_to_std_vector(const Eigen::VectorXd &eigen_vec)
    {
        return std::vector<double>(eigen_vec.data(), eigen_vec.data() + eigen_vec.size());
    }

    // std::vector转Eigen向量
    template <typename T>
    Eigen::VectorXd std_to_eigen_vector(const std::vector<T> &vec)
    {
        return Eigen::Map<const Eigen::VectorXd>(vec.data(), vec.size());
    }

    // 原生数组转std::vector
    template <typename T>
    std::vector<T> array_to_vector(const T *array, size_t size)
    {
        return std::vector<T>(array, array + size);
    }

    // 计算向量距离（L1范数）
    template <typename T>
    double vector_distance(const std::vector<T> &a, const std::vector<T> &b)
    {
        if (a.size() != b.size())
            return std::numeric_limits<double>::infinity();

        double distance = 0.0;
        for (size_t i = 0; i < a.size(); ++i)
        {
            distance += std::abs(a[i] - b[i]);
        }
        return distance;
    }

    // 矩阵伪逆（使用SVD）
    inline Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &matrix, double tolerance = 1e-8)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);

        const auto &singular_values = svd.singularValues();
        Eigen::VectorXd inv_singular_values(singular_values.size());

        for (int i = 0; i < singular_values.size(); ++i)
        {
            if (singular_values(i) > tolerance)
            {
                inv_singular_values(i) = 1.0 / singular_values(i);
            }
            else
            {
                inv_singular_values(i) = 0.0;
            }
        }

        return svd.matrixV() * inv_singular_values.asDiagonal() * svd.matrixU().transpose();
    }



inline double distance(const std::vector<double>& joints1, const std::vector<double>& joints2)
{
    if (joints1.size() != joints2.size()) {
        return INFINITY;
    }
    double sum = 0;
    for (int i = 0; i < joints1.size(); i++) {
        sum += std::abs(joints1[i] - joints2[i]);
    }
    return sum;
}

inline std::vector<double> tf2vec(const tf_t& tf)
{
    vec3_t trans = tf.translation();
    vec3_t eular = tf.rotation().eulerAngles(2, 1, 0).reverse();
    return std::vector<double>{trans.x(), trans.y(), trans.z(), eular.x(), eular.y(), eular.z()};
}

inline tf_t vec2tf(const std::vector<double>& vec)
{
    tf_t tf;
    tf.setIdentity();
    tf.translation() << vec[0], vec[1], vec[2];
    tf.rotate(Eigen::AngleAxisd(vec[5], vec3_t::UnitZ()));
    tf.rotate(Eigen::AngleAxisd(vec[4], vec3_t::UnitY()));
    tf.rotate(Eigen::AngleAxisd(vec[3], vec3_t::UnitX()));
    return tf;
}

inline std::vector<double> tf2tcp(const tf_t& tf, const double& ratio = 1.0)
{
    vec3_t trans = tf.translation() * ratio;
    Eigen::AngleAxisd aa(tf.rotation());
    vec3_t rv = aa.axis() * aa.angle();
    return std::vector<double>{trans.x(), trans.y(), trans.z(), rv.x(), rv.y(), rv.z()};
}

inline tf_t tcp2tf(const std::vector<double>& tcp, const double& ratio = 1.0)
{
    Eigen::Translation3d trans(tcp[0] * ratio, tcp[1] * ratio, tcp[2] * ratio);
    vec3_t rv(tcp[3], tcp[4], tcp[5]);
    double angle = rv.norm();
    if (is_near_zero(angle)) {
        return tf_t(trans);
    }
    else {
        rv /= angle;
        Eigen::AngleAxisd aa(angle, rv);
        return trans * aa;
    }
}

inline Eigen::Quaterniond axisAngleToQuaternion(const Eigen::Vector3d& axis, double angle_rad) {
    Eigen::Vector3d n = axis.normalized(); // 归一化旋转轴
    double half_angle = angle_rad / 2.0;
    double sin_half = sin(half_angle);
    double cos_half = cos(half_angle);

    Eigen::Quaterniond q;
    q.w() = cos_half;
    q.x() = n.x() * sin_half;
    q.y() = n.y() * sin_half;
    q.z() = n.z() * sin_half;
    // 注：单位四元数已自动满足 ||q||=1
    return q;
}

}
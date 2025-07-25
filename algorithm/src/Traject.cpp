#include "Traject.hpp"
#include <iostream>
namespace algorithms
{
    void QuinticTrajectory::addSegment(const TrajectoryPoint &start, const TrajectoryPoint &end, double duration)
    {
        segments_.push_back({start, end, duration});
    }

    void QuinticTrajectory::reSetSegment(double duration)
    {
        for (int i = 0; i < segments_.size(); i++)
        {
            segments_[i].duration = duration;
        }
    }

    std::vector<TrajectoryPoint> QuinticTrajectory::generate()
    {
        std::vector<TrajectoryPoint> full_trajectory;

        // 无过渡段时直接生成
        if (segments_.empty())
            return full_trajectory;
        if (segments_.size() == 1 || !arc_params_.enable)
        {
            return generateWithoutTransition();
        }

        // 带圆弧过渡的轨迹生成
        full_trajectory = generateWithArcTransition();
        return full_trajectory;
    }

    bool QuinticTrajectory::isTrajectorySafe(const std::vector<TrajectoryPoint> &traj)
    {
        for (const auto &point : traj)
        {
            if (!checkConstraints(point.velocity))
                return false;
        }
        return true;
    }

    std::vector<TrajectoryPoint> QuinticTrajectory::generateWithArcTransition()
    {
        std::vector<TrajectoryPoint> trajectory;
        // 首段轨迹（无过渡）
        generateSegmentTrajectory(segments_[0], trajectory);

        // 中间段：添加圆弧过渡
        for (size_t i = 1; i < segments_.size() - 1; ++i)
        {
            const auto &prev_seg = segments_[i - 1];
            const auto &curr_seg = segments_[i];
            const auto &next_seg = segments_[i + 1];

            // 计算过渡圆弧参数 [4,5](@ref)
            Eigen::Vector3d P0 = prev_seg.end.position;
            Eigen::Vector3d P1 = curr_seg.end.position; // 当前段终点（即过渡点）
            Eigen::Vector3d P2 = next_seg.end.position;

            // 计算过渡圆弧圆心和切点 [5](@ref)
            Eigen::Vector3d arc_center;
            Eigen::Vector3d tangent_in, tangent_out;
            computeArcTransition(P0, P1, P2, arc_center, tangent_in, tangent_out);

            // 生成圆弧段轨迹
            generateArcSegment(P1, tangent_in, tangent_out, arc_center, trajectory);

            // 添加当前段轨迹（从圆弧终点开始）
            Segment adjusted_seg = curr_seg;
            adjusted_seg.start.position = tangent_out;
            generateSegmentTrajectory(adjusted_seg, trajectory, true); // 跳过起点
        }

        // 末段轨迹（无过渡）
        generateSegmentTrajectory(segments_.back(), trajectory);
        return trajectory;
    }

    void QuinticTrajectory::generateArcSegment(
        const Eigen::Vector3d &P1,
        const Eigen::Vector3d &tangent_in,
        const Eigen::Vector3d &tangent_out,
        const Eigen::Vector3d &arc_center,
        std::vector<TrajectoryPoint> &trajectory)
    {
        // 计算圆弧几何参数
        Eigen::Vector3d start_dir = (tangent_in - arc_center).normalized();
        Eigen::Vector3d end_dir = (tangent_out - arc_center).normalized();
        Eigen::Vector3d rot_axis = start_dir.cross(end_dir).normalized();
        double rot_angle = std::acos(start_dir.dot(end_dir));

        // 圆弧插值点数（基于时间占比）
        int num_arc_points = static_cast<int>(arc_params_.blend_ratio * segments_[0].duration / time_step_);

        // 姿态插值（起点与终点四元数）[1](@ref)
        Eigen::Quaterniond q_start = trajectory.back().quaternion;
        Eigen::Quaterniond q_end = segments_[1].start.quaternion;

        // 生成圆弧点
        for (int i = 0; i <= num_arc_points; ++i)
        {
            double t_ratio = static_cast<double>(i) / num_arc_points;
            double angle = t_ratio * rot_angle;

            // 位置插值（旋转向量）
            Eigen::AngleAxisd rot(angle, rot_axis);
            Eigen::Vector3d pos = arc_center + rot * start_dir * (tangent_in - arc_center).norm();

            // 姿态插值（对数四元数 SLERP）[1](@ref)
            Eigen::Quaterniond quat = q_start.slerp(t_ratio, q_end);

            // 速度计算（切向速度）
            Eigen::Vector3d radial = (pos - arc_center).normalized();
            Eigen::Vector3d vel = radial.cross(rot_axis) * rot_angle / (num_arc_points * time_step_);

            // 加速度（向心加速度）
            Eigen::Vector3d acc = -radial * vel.squaredNorm() / (tangent_in - arc_center).norm();

            trajectory.push_back({pos, quat, vel, Eigen::Vector3d::Zero(), acc});
        }
    }

    void QuinticTrajectory::computeArcTransition(const Eigen::Vector3d &P0, const Eigen::Vector3d &P1, const Eigen::Vector3d &P2, Eigen::Vector3d &arc_center, Eigen::Vector3d &tangent_in, Eigen::Vector3d &tangent_out)
    {
        // 向量计算 [5](@ref)
        Eigen::Vector3d V1 = (P1 - P0).normalized(); // 前一方向
        Eigen::Vector3d V2 = (P2 - P1).normalized(); // 后一方向
        Eigen::Vector3d bisector = (V1 + V2).normalized();

        // 动态半径（避免小弧问题）[4](@ref)
        double theta = std::acos(V1.dot(V2)); // 路径夹角
        double dynamic_radius = std::min(arc_params_.radius, 0.5 * (P1 - P0).norm() * std::tan(theta / 2));

        // 圆心位置 [5](@ref)
        double d = dynamic_radius / std::sin(theta / 2);
        arc_center = P1 - bisector * d;

        // 切点计算（沿方向偏移）[6](@ref)
        tangent_in = P1 - V1 * dynamic_radius / std::tan(theta / 2);
        tangent_out = P1 + V2 * dynamic_radius / std::tan(theta / 2);
    }

    // 生成单段轨迹（无过渡）
    void QuinticTrajectory::generateSegmentTrajectory(const Segment &seg, std::vector<TrajectoryPoint> &trajectory, bool skip_start)
    {
        auto pos_coeffs = computeCoefficients(
            seg.start.position, seg.end.position,
            seg.start.velocity, seg.end.velocity,
            seg.start.acceleration, seg.end.acceleration,
            seg.duration);

        Eigen::Quaterniond q_start = seg.start.quaternion.normalized();
        Eigen::Quaterniond q_end = seg.end.quaternion.normalized();
        int points = static_cast<int>(seg.duration / time_step_);

        for (int j = skip_start ? 1 : 0; j <= points; ++j)
        {
            double t = j * time_step_;
            double u = t / seg.duration;

            TrajectoryPoint point;
            point.position = evalPolynomial(pos_coeffs, t);
            point.velocity = evalPolynomial(pos_coeffs, t, 1);
            point.acceleration = evalPolynomial(pos_coeffs, t, 2);
            point.quaternion = q_start.slerp(u, q_end);

            // 角速度差分计算
            if (!trajectory.empty())
            {
                Eigen::Quaterniond dq = point.quaternion * trajectory.back().quaternion.inverse();
                point.angular_vel = 2.0 * dq.vec() / time_step_;
            }
            trajectory.push_back(point);
        }
    }

    std::vector<TrajectoryPoint> QuinticTrajectory::generateWithoutTransition()
    {
        std::vector<TrajectoryPoint> full_trajectory;
        double current_time = 0.0;
        for (size_t i = 0; i < segments_.size(); ++i)
        {
            const auto &seg = segments_[i];
            const int points_per_seg = static_cast<int>(seg.duration / time_step_);

            // 计算位置和姿态的插值参数
            auto pos_coeffs = computeCoefficients(seg.start.position, seg.end.position,
                                                  seg.start.velocity, seg.end.velocity,
                                                  seg.start.acceleration, seg.end.acceleration,
                                                  seg.duration);

            // 姿态插值（四元数SLERP）
            Eigen::Quaterniond q_start = seg.start.quaternion.normalized();
            Eigen::Quaterniond q_end = seg.end.quaternion.normalized();

            for (int j = 0; j <= points_per_seg; ++j)
            {
                double t = j * time_step_;
                double u = t / seg.duration; // 归一化时间 [0,1]

                // 位置插值（五次多项式）
                Eigen::Vector3d pos = evalPolynomial(pos_coeffs, t);
                Eigen::Vector3d vel = evalPolynomial(pos_coeffs, t, 1);
                Eigen::Vector3d acc = evalPolynomial(pos_coeffs, t, 2);

                // 姿态插值（球面线性插值）
                Eigen::Quaterniond quat = q_start.slerp(u, q_end);

                // 角速度近似（有限差分）
                Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero();
                Eigen::Vector3d angular_acc = Eigen::Vector3d::Zero();
                if (j > 0)
                {
                    Eigen::Quaterniond dq = quat * full_trajectory.back().quaternion.inverse();
                    angular_vel = 2.0 * dq.vec() / time_step_; // 小角度近似
                    angular_acc = angular_vel - full_trajectory.back().angular_vel / time_step_;
                }
                tf_t waypoint;
                waypoint.translation() = pos;
                Matrix3d rot =  quat.toRotationMatrix();
                waypoint.linear()= rot;
                // 保存轨迹点
                full_trajectory.push_back({pos, quat, vel, angular_vel, acc, angular_acc,waypoint});
            }

            // 更新当前时间（用于下一段起点）
            current_time += seg.duration;
        }
        return full_trajectory;
    }

    bool QuinticTrajectory::checkConstraints(const Eigen::Vector3d &vel)
    {
        for (int i = 0; i < 3; i++)
        {
            if (std::fabs(vel[i]) > MAX_VELOCITY)
            {
                return false;
            }
        }
        return true;
    }

    std::array<Eigen::Vector3d, 6> QuinticTrajectory::computeCoefficients(
        const Eigen::Vector3d &q0, const Eigen::Vector3d &q1,
        const Eigen::Vector3d &v0, const Eigen::Vector3d &v1,
        const Eigen::Vector3d &a0, const Eigen::Vector3d &a1,
        double T)
    {
        std::array<Eigen::Vector3d, 6> coeffs;
        double T2 = T * T;
        double T3 = T2 * T;
        double T4 = T3 * T;
        double T5 = T4 * T;

        // 位置系数
        coeffs[0] = q0;
        coeffs[1] = v0;
        coeffs[2] = 0.5 * a0;
        coeffs[3] = (20 * (q1 - q0) - (8 * v1 + 12 * v0) * T - (3 * a0 - a1) * T2) / (2 * T3);
        coeffs[4] = (-30 * (q1 - q0) + (14 * v1 + 16 * v0) * T + (3 * a0 - 2 * a1) * T2) / (2 * T4);
        coeffs[5] = (12 * (q1 - q0) - 6 * (v1 + v0) * T + (a1 - a0) * T2) / (2 * T5);

        return coeffs;
    }

    // 计算多项式值（或导数）
    Eigen::Vector3d QuinticTrajectory::evalPolynomial(const std::array<Eigen::Vector3d, 6> &coeffs, double t, int order)
    {
        Eigen::Vector3d result = Eigen::Vector3d::Zero();
        if (order == 0)
        { // 位置
            result = coeffs[0] + coeffs[1] * t + coeffs[2] * t * t + coeffs[3] * t * t * t +
                     coeffs[4] * t * t * t * t + coeffs[5] * t * t * t * t * t;
        }
        else if (order == 1)
        { // 速度
            result = coeffs[1] + 2 * coeffs[2] * t + 3 * coeffs[3] * t * t +
                     4 * coeffs[4] * t * t * t + 5 * coeffs[5] * t * t * t * t;
        }
        else if (order == 2)
        { // 加速度
            result = 2 * coeffs[2] + 6 * coeffs[3] * t + 12 * coeffs[4] * t * t + 20 * coeffs[5] * t * t * t;
        }
        return result;
    }

    void LinearInterpolator::addWaypoint(const TrajectoryPoint &pose)
    {
        waypoints_.push_back(pose);
    }

    void LinearInterpolator::addSegment(const TrajectoryPoint &start, const TrajectoryPoint &end, double duration)
    {
        segments_.push_back({start, end, duration});
        segment_times_.push_back(duration);
    }

    // 设置总时间或各段时间（可重载）
    void LinearInterpolator::setTotalTime(double total_time)
    {
        total_time_ = total_time;
    }
    void LinearInterpolator::setSegmentTimes(const std::vector<double> &segment_times)
    {
        segment_times_ = segment_times;
    }

    // 设置梯形速度参数（加速时间占比）
    void LinearInterpolator::setTrapezoidProfile(double accel_ratio)
    {
        accel_ratio_ = accel_ratio;
    }

    // 生成多段轨迹（返回所有插值点）
    std::vector<TrajectoryPoint> LinearInterpolator::generate()
    {
        //    if (waypoints_.size() < 2) {
        //        throw std::runtime_error("至少需要2个路径点");
        //    }

        //    // 计算各段时间（若未指定则平均分配）
        //    if (segment_times_.empty()) {
        //        segment_times_ = std::vector<double>(waypoints_.size() - 1,
        //                           total_time_ / (waypoints_.size() - 1));
        //    }

        //    std::vector<TrajectoryPoint> trajectory;
        //    for (size_t seg = 0; seg < waypoints_.size() - 1; ++seg) {
        //        const auto& start = waypoints_[seg];
        //        const auto& end = waypoints_[seg + 1];
        //        double seg_time = segment_times_[seg];
        //        const int points_per_segment = static_cast<int>(seg_time / time_step_) + 1;
        //        // 单段插值（调用内部函数）
        //        auto seg_traj = interpolateSegment(start, end, seg_time, points_per_segment);
        //        // 避免重复添加终点（下一段的起点）
        //        if (!trajectory.empty()) seg_traj.erase(seg_traj.begin());
        //        trajectory.insert(trajectory.end(), seg_traj.begin(), seg_traj.end());
        //    }
        //    return trajectory;
        std::vector<TrajectoryPoint> full_trajectory;
        if (segments_.size() < 1)
        {
            return full_trajectory;
        }
        if (segment_times_.empty())
        {
            segment_times_ = std::vector<double>(waypoints_.size() - 1,
                                                 total_time_ / (waypoints_.size() - 1));
        }
        for (size_t seg = 0; seg < segments_.size(); seg++)
        {
            const auto &start = segments_[seg].start;
            const auto &end = segments_[seg].end;
            double seg_time = segment_times_[seg];
            const int points_per_segment = static_cast<int>(seg_time / time_step_) + 1;
            // 单段插值（调用内部函数）
            auto seg_traj = interpolateSegment(start, end, seg_time, points_per_segment);
            // 避免重复添加终点（下一段的起点）
            if (!full_trajectory.empty())
                seg_traj.erase(seg_traj.begin());
            full_trajectory.insert(full_trajectory.end(), seg_traj.begin(), seg_traj.end());
        }
        return full_trajectory;
    }

    std::vector<TrajectoryPoint> LinearInterpolator::interpolateSegment(const TrajectoryPoint &start, const TrajectoryPoint &end,
                                                                        double seg_time, int num_points)
    {
        std::vector<TrajectoryPoint> segment;
        const double dt = seg_time / (num_points - 1); // 时间步长

        // 梯形速度参数计算
        double accel_time = seg_time * accel_ratio_;
        double cruise_time = seg_time - 2 * accel_time;

        for (int i = 0; i < num_points; ++i)
        {
            double t = static_cast<double>(i) / (num_points - 1);
            double s = computeTimeScaling(t, seg_time, accel_time, cruise_time);
            TrajectoryPoint current_pose = interpolatePose(start, end, s);

            // 第一帧初始化运动学量为0
            // 首帧初始化
            if (i == 0)
                initPoseKinematics(current_pose);
            else
                updateKinematics(current_pose, segment.back(), time_step_);
            segment.push_back(current_pose);
        }
        return segment;
    }

    std::vector<TrajectoryPoint> LinearInterpolator::interpolateSegmentSCurve(const TrajectoryPoint &start, const TrajectoryPoint &end, double T)
    {
        std::vector<TrajectoryPoint> segment;
        Eigen::Vector3d displacement = end.position - start.position;
        double distance = displacement.norm();

        // 1. 计算S型曲线时间参数 [10](@ref)
        double Tj1 = std::min(sqrt(max_accel_ / max_jerk_), T / 2); // 加加速段时间
        double Ta = 2 * Tj1;                                        // 总加速时间
        double Tv = T - 2 * Ta;                                     // 匀速段时间（可能为负）

        // 若无法达到匀速段，重新规划为三角形S曲线
        if (Tv < 0)
        {
            Tj1 = T / 2;
            Ta = T;
            Tv = 0;
        }

        // 2. 生成时间序列
        const int num_points = static_cast<int>(T / 0.002) + 1; // 2ms周期
        for (int i = 0; i < num_points; ++i)
        {
            double t = static_cast<double>(i) * 0.002; // 当前时间
            double s = computeSCurveScaling(t, T, Tj1, Ta, Tv);
            TrajectoryPoint pose = interpolatePose(start, end, s);

            // 首帧初始化
            if (i == 0)
                initPoseKinematics(pose);
            else
                updateKinematics(pose, segment.back(), 0.002);

            segment.push_back(pose);
        }
        return segment;
    }

    // 梯形速度规划：归一化位置计算 [0,1]
    double LinearInterpolator::computeTimeScaling(double t, double seg_time,
                                                  double accel_time, double cruise_time) const
    {
        double t_abs = t * seg_time;
        if (t_abs <= accel_time)
        {
            return 0.5 * std::pow(t_abs / accel_time, 2); // 加速段
        }
        else if (t_abs <= accel_time + cruise_time)
        {
            return 0.5 + (t_abs - accel_time) / seg_time; // 匀速段
        }
        else
        {
            double t_decel = t_abs - (accel_time + cruise_time);
            double ratio = t_decel / accel_time;
            return 1.0 - 0.5 * std::pow(1 - ratio, 2); // 减速段
        }
    }

    double LinearInterpolator::computeSCurveScaling(double t, double T, double Tj, double Ta, double Tv) const
    {
        const double t1 = Tj, t2 = Ta - Tj, t3 = Ta, t4 = Ta + Tv, t5 = T - Tj, t6 = T - Tj;
        const double j_max = (Tj > 0) ? (Ta / (2 * Tj * Tj)) : 0;
        const double a_max = j_max * Tj;
        const double v_max = a_max * (Ta - Tj);

        if (t <= t1)
        {
            return (j_max * t * t * t) / 6.0;
        }
        else if (t <= t2)
        {
            double dt = t - t1;
            double s1 = (j_max * t1 * t1 * t1) / 6.0;
            double v1 = (j_max * t1 * t1) / 2.0;
            return s1 + v1 * dt + 0.5 * a_max * dt * dt;
        }
        else if (t <= t3)
        {
            double dt = t - t2;
            double s2 = (j_max * t1 * t1 * t1) / 6.0 +
                        (j_max * t1 * t1 * (t2 - t1)) / 2.0 +
                        0.5 * a_max * (t2 - t1) * (t2 - t1);
            double v2 = v_max;
            return s2 + v2 * dt + 0.5 * a_max * dt * dt - (j_max * dt * dt * dt) / 6.0;
        }
        else if (t <= t4)
        {
            double dt = t - t3;
            double s3 = v_max * (t3 - t1) + (j_max * t1 * t1 * t1) / 6.0;
            return s3 + v_max * dt;
        }
        else if (t <= t5)
        {
            // 对称阶段3（加减速）
            double dt = t - t4;
            double s4 = (j_max * t1 * t1 * t1) / 6.0 +
                        (j_max * t1 * t1 * (t2 - t1)) / 2.0 +
                        0.5 * a_max * (t2 - t1) * (t2 - t1);
            double v4 = v_max;
            return s4 + v4 * dt + 0.5 * a_max * dt * dt - (j_max * dt * dt * dt) / 6.0;
        }
        else if (t <= t6)
        {
            // 对称阶段2（匀减速）
            double dt = t - t5;
            double s5 = (j_max * t1 * t1 * t1) / 6.0;
            double v5 = (j_max * t1 * t1) / 2.0;
            return s5 + v5 * dt + 0.5 * a_max * dt * dt;
        }
        else
        {
            // 对称阶段1（减减速）
            double dt = t - t6;
            double s6 = 1.0 - (j_max * dt * dt * dt) / 6.0;
            return s6;
        }
    }

    // 位姿插值核心
    TrajectoryPoint LinearInterpolator::interpolatePose(const TrajectoryPoint &start, const TrajectoryPoint &end, double s) const
    {
        TrajectoryPoint pose;
        // 位置线性插值: P(s) = P_start + s * (P_end - P_start)
        pose.position = start.position + s * (end.position - start.position);

        // 姿态球面线性插值 (SLERP)
        double dot = start.quaternion.dot(end.quaternion);
        double sign = (dot >= 0) ? 1.0 : -1.0; // 处理四元数方向
        Eigen::Quaterniond adjusted_end = Eigen::Quaterniond(sign * end.quaternion.coeffs());
        pose.quaternion = start.quaternion.slerp(s, adjusted_end);
        return pose;
    }

    void LinearInterpolator::initPoseKinematics(TrajectoryPoint &pose)
    {
        pose.velocity.setZero();
        pose.angular_vel.setZero();
        pose.acceleration.setZero();
    }

    void LinearInterpolator::updateKinematics(TrajectoryPoint &current, const TrajectoryPoint &prev, double dt)
    {
        current.velocity = (current.position - prev.position) / dt;
        current.acceleration = (current.velocity - prev.velocity) / dt;
        //    current.linear_jerk = (current.linear_acceleration - prev.linear_acceleration) / dt;

        // 角速度 & 角加速度（四元数微分）
        Eigen::Quaterniond dq = current.quaternion * prev.quaternion.inverse();
        dq.normalize();
        current.angular_vel = 2.0 * Eigen::Vector3d(dq.x(), dq.y(), dq.z()) / dt;
        //    current.angular_acceleration = (current.angular_velocity - prev.angular_velocity) / dt;
    }

    void SplinePoseInterpolator::addWaypoint(const Eigen::Vector3d &pos, const Eigen::Quaterniond &quat, double t)
    {
        if (!times_.empty() && t <= times_.back())
        {
            throw std::invalid_argument("时间必须严格递增");
        }
        times_.push_back(t);
        positions_.push_back(pos);
        orientations_.push_back(quat.normalized()); // 单位化四元数
    }

    void SplinePoseInterpolator::compute()
    {
        // 1. 校验数据
        if (times_.size() < 2)
            throw std::runtime_error("至少需要2个路径点");

        // 2. 计算位置样条系数（同之前的实现）
        computePositionSpline();

        // 3. 姿态插值无需预计算系数（SLERP 实时计算）
    }

    std::vector<TrajectoryPoint> SplinePoseInterpolator::generate()
    {
        compute(); // 确保系数已计算

        std::vector<TrajectoryPoint> trajectory;
        double t_start = times_.front();
        double t_end = times_.back();

        // 首点初始化
        TrajectoryPoint prev_pt;
        prev_pt.position = positions_.front();
        prev_pt.quaternion = orientations_.front();
        prev_pt.velocity.setZero();
        prev_pt.acceleration.setZero();
        trajectory.push_back(prev_pt);

        // 按周期插值
        for (double t = t_start + step_time_; t <= t_end; t += step_time_)
        {
            TrajectoryPoint pt;

            // === 1. 位置插值（三次样条） ===
            auto pos_it = std::upper_bound(times_.begin(), times_.end(), t);
            int seg_idx = std::distance(times_.begin(), pos_it) - 1;
            double t_seg = t - times_[seg_idx];
            double h = times_[seg_idx + 1] - times_[seg_idx];
            pt.position = a_[seg_idx] + b_[seg_idx] * t_seg +
                          c_[seg_idx] * t_seg * t_seg +
                          d_[seg_idx] * t_seg * t_seg * t_seg;

            // === 2. 姿态插值（四元数 SLERP） ===
            double t_normalized = (t - times_[seg_idx]) / h; // 归一化时间 [0,1]
            pt.quaternion = orientations_[seg_idx].slerp(t_normalized, orientations_[seg_idx + 1]);

            // === 3. 差分法计算速度/加速度 ===
            pt.velocity = (pt.position - prev_pt.position) / step_time_;
            pt.acceleration = (pt.velocity - prev_pt.velocity) / step_time_;
            Eigen::Quaterniond dq = pt.quaternion * prev_pt.quaternion.conjugate();
            pt.angular_vel = 2.0 * Eigen::Vector3d(dq.x(), dq.y(), dq.z()) / step_time_;

            trajectory.push_back(pt);
            prev_pt = pt;
        }
        return trajectory;
    }

    void SplinePoseInterpolator::computePositionSpline()
    {
        const int n = times_.size() - 1;
        a_ = positions_;
        b_.resize(n + 1, Eigen::Vector3d::Zero());
        c_.resize(n + 1, Eigen::Vector3d::Zero());
        d_.resize(n + 1, Eigen::Vector3d::Zero());

        // 构建三对角矩阵（自然边界条件）
        Eigen::VectorXd h(n);
        for (int i = 0; i < n; ++i)
            h[i] = times_[i + 1] - times_[i];

        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n + 1, n + 1);
        Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(n + 1, 3);

        A(0, 0) = 1.0;
        A(n, n) = 1.0;
        for (int i = 1; i < n; ++i)
        {
            A(i, i - 1) = h[i - 1];
            A(i, i) = 2.0 * (h[i - 1] + h[i]);
            A(i, i + 1) = h[i];
            rhs.row(i) = 3.0 * ((positions_[i + 1] - positions_[i]) / h[i] -
                                (positions_[i] - positions_[i - 1]) / h[i - 1]);
        }

        // 求解三对角矩阵（追赶法）
        for (int dim = 0; dim < 3; ++dim)
        {
            Eigen::VectorXd c_tmp(n + 1);
            Eigen::VectorXd delta(n + 1), lambda(n + 1);

            // 前向消元
            delta[0] = A(0, 0);
            lambda[0] = rhs(0, dim) / delta[0];
            for (int i = 1; i <= n; ++i)
            {
                delta[i] = A(i, i) - A(i, i - 1) * A(i - 1, i) / delta[i - 1];
                lambda[i] = (rhs(i, dim) - A(i, i - 1) * lambda[i - 1]) / delta[i];
            }

            // 回代求解
            c_tmp[n] = lambda[n];
            for (int i = n - 1; i >= 0; --i)
            {
                c_tmp[i] = lambda[i] - (A(i, i + 1) / delta[i]) * c_tmp[i + 1];
            }
            for (int i = 0; i <= n; ++i)
                c_[i][dim] = c_tmp[i];
        }

        // 计算 b, d 系数
        for (int i = 0; i < n; ++i)
        {
            b_[i] = (positions_[i + 1] - positions_[i]) / h[i] -
                    h[i] * (2 * c_[i] + c_[i + 1]) / 3.0;
            d_[i] = (c_[i + 1] - c_[i]) / (3.0 * h[i]);
        }
    }
}
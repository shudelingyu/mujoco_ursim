#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
namespace algorithms
{
    inline constexpr double MAX_VELOCITY = 0.1;
    inline constexpr double MAX_ACCELERATION = 0.5;

    struct TrajectoryPoint
    {
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Quaterniond quaternion = Eigen::Quaterniond::Identity();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();     // 线速度 (m/s)
        Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero();  // 角速度 (rad/s)
        Eigen::Vector3d acceleration = Eigen::Vector3d::Zero(); // 线加速度 (m/s²)
        Eigen::Vector3d angular_acc = Eigen::Vector3d::Zero();  // 线加速度 (m/s²)
    };

    class QuinticTrajectory
    {
    public:
        QuinticTrajectory() {}
        ~QuinticTrajectory() {}

    public:
        struct ArcTransition
        {
            double radius = 0.05;     // 默认过渡半径
            double blend_ratio = 0.3; // 过渡区时间占比
            bool enable = false;      // 是否启用圆弧过渡
        };

        // 添加一段轨迹（起点+终点+时间）
        void addSegment(const TrajectoryPoint &start, const TrajectoryPoint &end, double duration);
        void setArcTransition(const ArcTransition &params)
        {
            arc_params_ = params;
        }
        void reSetSegment(double duration);
        // 生成完整轨迹（多段拼接）
        std::vector<TrajectoryPoint> generate();
        //    检查轨迹是否安全
        bool isTrajectorySafe(const std::vector<TrajectoryPoint> &traj);

    private:
        struct Segment
        {
            TrajectoryPoint start;
            TrajectoryPoint end;
            double duration;
        };

    private:
        std::vector<TrajectoryPoint> generateWithArcTransition();
        void generateArcSegment(const Eigen::Vector3d &P1,
                                const Eigen::Vector3d &tangent_in,
                                const Eigen::Vector3d &tangent_out,
                                const Eigen::Vector3d &arc_center,
                                std::vector<TrajectoryPoint> &trajectory);
        void computeArcTransition(const Eigen::Vector3d &P0,
                                  const Eigen::Vector3d &P1,
                                  const Eigen::Vector3d &P2,
                                  Eigen::Vector3d &arc_center,
                                  Eigen::Vector3d &tangent_in,
                                  Eigen::Vector3d &tangent_out);
        void generateSegmentTrajectory(const Segment &seg, std::vector<TrajectoryPoint> &trajectory, bool skip_start = false);
        std::vector<TrajectoryPoint> generateWithoutTransition();
        // 计算五次多项式系数 (a0~a5)
        std::array<Eigen::Vector3d, 6> computeCoefficients(
            const Eigen::Vector3d &q0, const Eigen::Vector3d &q1,
            const Eigen::Vector3d &v0, const Eigen::Vector3d &v1,
            const Eigen::Vector3d &a0, const Eigen::Vector3d &a1,
            double T);

        // 计算多项式值（或导数）
        Eigen::Vector3d evalPolynomial(const std::array<Eigen::Vector3d, 6> &coeffs, double t, int order = 0);
        //    检查轨迹点安全性
        bool checkConstraints(const Eigen::Vector3d &vel);

    private:
        std::vector<Segment> segments_;
        ArcTransition arc_params_;
        double time_step_ = 0.002; // 10ms 时间步长
    };

    class LinearInterpolator
    {
    public:
        // 位姿结构体：位置（XYZ） + 姿态（四元数）

        // 添加路径点（位置+姿态）
        void addWaypoint(const TrajectoryPoint &pose);
        void addSegment(const TrajectoryPoint &start, const TrajectoryPoint &end, double duration);
        // 设置总时间或各段时间（可重载）
        void setTotalTime(double total_time);
        void setSegmentTimes(const std::vector<double> &segment_times);
        // 设置梯形速度参数（加速时间占比）
        void setTrapezoidProfile(double accel_ratio = 0.3);

        void setSProfile(double max_accel, double max_decel, double max_jerk)
        {
            max_accel_ = max_accel;
            max_decel_ = max_decel;
            max_jerk_ = max_jerk;
        }
        // 生成多段轨迹（返回所有插值点）
        std::vector<TrajectoryPoint> generate();

    private:
        struct Segment
        {
            TrajectoryPoint start;
            TrajectoryPoint end;
            double duration;
        };

    private:
        // 单段直线插值（LERP + SLERP + 梯形速度规划）
        std::vector<TrajectoryPoint> interpolateSegment(const TrajectoryPoint &start, const TrajectoryPoint &end, double seg_time, int num_points);
        std::vector<TrajectoryPoint> interpolateSegmentSCurve(const TrajectoryPoint &start, const TrajectoryPoint &end, double T);
        // 梯形速度规划：归一化位置计算 [0,1]
        double computeTimeScaling(double t, double seg_time, double accel_time, double cruise_time) const;
        // S形速度规划
        double computeSCurveScaling(double t, double T, double Tj, double Ta, double Tv) const;
        // 位姿插值核心
        TrajectoryPoint interpolatePose(const TrajectoryPoint &start, const TrajectoryPoint &end, double s) const;
        void initPoseKinematics(TrajectoryPoint &pose);
        void updateKinematics(TrajectoryPoint &current, const TrajectoryPoint &prev, double dt);

    private:
        std::vector<TrajectoryPoint> waypoints_; // 路径点序列
        std::vector<Segment> segments_;
        std::vector<double> segment_times_; // 各段时间长度
        double total_time_ = 1.0;           // 总时间（若未分段设置）
        double accel_ratio_ = 0.3;          // 梯形加速段时间占比
        double time_step_ = 0.002;          // 10ms 时间步长
        double max_accel_ = 1.0;            // 最大加速度 (m/s²)
        double max_decel_ = 1.0;            // 最大减速度 (m/s²)
        double max_jerk_ = 2.0;             // 最大加加速度 (m/s³)
    };

    class SplinePoseInterpolator
    {
    public:
        // 添加路径点（时间 + 位置 + 姿态）
        void addWaypoint(const Eigen::Vector3d &pos, const Eigen::Quaterniond &quat, double t);

        // 计算样条系数（位置）与准备姿态插值
        void compute();

        // 生成轨迹（按固定周期输出）
        std::vector<TrajectoryPoint> generate();

    private:
        std::vector<double> times_;                    // 时间序列
        std::vector<Eigen::Vector3d> positions_;       // 位置序列
        std::vector<Eigen::Quaterniond> orientations_; // 姿态序列（单位四元数）
        double step_time_ = 0.002;

        // 位置样条系数
        std::vector<Eigen::Vector3d> a_, b_, c_, d_;

        // 位置样条系数计算（同之前实现）
        void computePositionSpline();
    };

}

#include <chrono>
#include <cmath>
#include <array>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include <traj_utils/msg/bspline.hpp>

using namespace std::chrono_literals;

class BsplinePx4Bridge : public rclcpp::Node
{
public:
    BsplinePx4Bridge()
    : Node("bspline_px4_bridge")
    {
        // -------------------------
        // 参数
        // -------------------------
        this->declare_parameter<double>("takeoff_x", 0.0);
        this->declare_parameter<double>("takeoff_y", 0.0);
        this->declare_parameter<double>("takeoff_z", 2.0);   // ENU / odom_world, 正数向上

        this->declare_parameter<double>("reach_thresh", 0.35);
        this->declare_parameter<double>("waypoint_timeout", 5.0);

        this->declare_parameter<bool>("auto_arm", true);

        // 如果你的 px4_odom_tf 里做了 x/y 交换，这里就设 true。
        // 如果 /odom_world.x = px4.x, /odom_world.y = px4.y，就设 false。
        this->declare_parameter<bool>("swap_xy", true);

        this->get_parameter("takeoff_x", takeoff_x_);
        this->get_parameter("takeoff_y", takeoff_y_);
        this->get_parameter("takeoff_z", takeoff_z_);

        this->get_parameter("reach_thresh", reach_thresh_);
        this->get_parameter("waypoint_timeout", waypoint_timeout_);
        this->get_parameter("auto_arm", auto_arm_);
        this->get_parameter("swap_xy", swap_xy_);

        // -------------------------
        // QoS
        // PX4 in/out 通常使用 best_effort
        // EGO Planner 的 bspline/odom 用 reliable
        // -------------------------
        auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        offboard_control_mode_publisher_ =
            create_publisher<px4_msgs::msg::OffboardControlMode>(
                "/fmu/in/offboard_control_mode",
                px4_qos);

        trajectory_setpoint_publisher_ =
            create_publisher<px4_msgs::msg::TrajectorySetpoint>(
                "/fmu/in/trajectory_setpoint",
                px4_qos);

        vehicle_command_publisher_ =
            create_publisher<px4_msgs::msg::VehicleCommand>(
                "/fmu/in/vehicle_command",
                px4_qos);

        timesync_status_subscriber_ =
            create_subscription<px4_msgs::msg::TimesyncStatus>(
                "/fmu/out/timesync_status",
                px4_qos,
                std::bind(
                    &BsplinePx4Bridge::timesyncStatusCallback,
                    this,
                    std::placeholders::_1));

        odom_subscriber_ =
            create_subscription<nav_msgs::msg::Odometry>(
                "/odom_world",
                reliable_qos,
                std::bind(
                    &BsplinePx4Bridge::odomCallback,
                    this,
                    std::placeholders::_1));

        bspline_subscriber_ =
            create_subscription<traj_utils::msg::Bspline>(
                "/planning/bspline",
                reliable_qos,
                std::bind(
                    &BsplinePx4Bridge::bsplineCallback,
                    this,
                    std::placeholders::_1));
        goal_subscriber_ =
            create_subscription<geometry_msgs::msg::PoseStamped>(
                "/goal_pose",
                reliable_qos,
                std::bind(
                    &BsplinePx4Bridge::goalCallback,
                    this,
                    std::placeholders::_1));
        // 100ms 一次，10Hz 控制频率，和你的 go/circle 节点一致
        timer_ =
            create_wall_timer(
                100ms,
                std::bind(&BsplinePx4Bridge::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "bspline_px4_bridge started.");
        RCLCPP_INFO(this->get_logger(), "takeoff target ENU: x=%.2f y=%.2f z=%.2f",
                    takeoff_x_, takeoff_y_, takeoff_z_);
    }

private:
    enum class ControlState
    {
        TAKEOFF_HOLD,
        WAIT_BSPLINE,
        TRACK_BSPLINE,
        HOLD
    };

    // -------------------------
    // ROS
    // -------------------------
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
        offboard_control_mode_publisher_;

    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
        trajectory_setpoint_publisher_;

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
        vehicle_command_publisher_;

    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr
        timesync_status_subscriber_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odom_subscriber_;

    rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr
        bspline_subscriber_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        goal_subscriber_;

    // -------------------------
    // PX4 time sync
    // -------------------------
    bool has_timesync_ = false;
    int64_t timesync_offset_us_ = 0;

    // -------------------------
    // 当前 odom_world 位姿，ENU，z 正向上
    // -------------------------
    bool has_odom_ = false;
    double odom_x_ = 0.0;
    double odom_y_ = 0.0;
    double odom_z_ = 0.0;

    // -------------------------
    // 参数
    // -------------------------
    double takeoff_x_ = 0.0;
    double takeoff_y_ = 0.0;
    double takeoff_z_ = 2.0;

    double reach_thresh_ = 0.35;
    double waypoint_timeout_ = 5.0;

    bool auto_arm_ = true;
    bool swap_xy_ = true;

    // -------------------------
    // 状态机
    // -------------------------
    ControlState state_ = ControlState::TAKEOFF_HOLD;
    uint64_t offboard_setpoint_counter_ = 0;

    // 当前悬停点，ENU
    double hold_x_ = 0.0;
    double hold_y_ = 0.0;
    double hold_z_ = 2.0;

    bool has_goal_ = false;
    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
    double goal_z_ = 2.0;
    // -------------------------
    // B 样条轨迹数据，ENU
    // -------------------------
    bool has_active_bspline_ = false;

    // 控制点，不是航点
    std::vector<std::array<double, 3>> control_points_;

    // B 样条节点向量
    std::vector<double> knots_;

    // B 样条 degree，EGO Planner 里 order 通常是 3，也就是三次 B 样条
    int bspline_degree_ = 3;

    // 轨迹参数范围
    double traj_u_start_ = 0.0;
    double traj_u_end_ = 0.0;

    // 本节点收到这条轨迹的时间，用它作为执行起点
    rclcpp::Time traj_receive_time_;
    // -------------------------
    // 时间戳
    // -------------------------
    uint64_t timestamp()
    {
        auto now_us = this->get_clock()->now().nanoseconds() / 1000;

        if (has_timesync_)
        {
            return static_cast<uint64_t>(now_us + timesync_offset_us_);
        }

        return static_cast<uint64_t>(now_us);
    }

    void timesyncStatusCallback(
        const px4_msgs::msg::TimesyncStatus::SharedPtr msg)
    {
        timesync_offset_us_ = msg->estimated_offset;
        has_timesync_ = true;
    }

    // -------------------------
    // odom 回调
    // -------------------------
    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_x_ = msg->pose.pose.position.x;
        odom_y_ = msg->pose.pose.position.y;
        odom_z_ = msg->pose.pose.position.z;

        has_odom_ = true;
    }

    void goalCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;
        goal_z_ = msg->pose.position.z;

        has_goal_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Received goal_pose: x=%.2f y=%.2f z=%.2f",
            goal_x_,
            goal_y_,
            goal_z_);
    }
    // -------------------------
    // bspline 回调
    // 第一版：不做严格 B 样条求值，只把 pos_pts 当成粗略目标点序列
    // -------------------------
    void bsplineCallback(
        const traj_utils::msg::Bspline::SharedPtr msg)
    {
        if (msg->pos_pts.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty bspline pos_pts.");
            return;
        }

        if (msg->knots.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty bspline knots.");
            return;
        }

        std::vector<std::array<double, 3>> new_control_points;

        for (const auto &p : msg->pos_pts)
        {
            if (!std::isfinite(p.x) ||
                !std::isfinite(p.y) ||
                !std::isfinite(p.z))
            {
                continue;
            }

            if (p.z < 0.2)
            {
                continue;
            }

            new_control_points.push_back({p.x, p.y, p.z});
        }

        if (new_control_points.empty())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "All new bspline control points were filtered. Keep current state.");
            return;
        }

        int degree = msg->order;

        if (degree <= 0)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid bspline order=%d.",
                degree);
            return;
        }

        int n = static_cast<int>(new_control_points.size()) - 1;

        if (static_cast<int>(msg->knots.size()) < n + degree + 2)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid bspline: control_points=%zu, degree=%d, knots=%zu",
                new_control_points.size(),
                degree,
                msg->knots.size());
            return;
        }

        control_points_ = new_control_points;
        knots_ = msg->knots;
        bspline_degree_ = degree;

        traj_u_start_ = knots_[bspline_degree_];
        traj_u_end_ = knots_[n + 1];

        if (traj_u_end_ <= traj_u_start_)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid bspline duration: u_start=%.3f, u_end=%.3f",
                traj_u_start_,
                traj_u_end_);
            return;
        }

        traj_receive_time_ = this->now();
        has_active_bspline_ = true;
        state_ = ControlState::TRACK_BSPLINE;

        RCLCPP_INFO(
            this->get_logger(),
            "Received bspline traj_id=%d, degree=%d, control_points=%zu, knots=%zu, u=[%.3f, %.3f]",
            msg->traj_id,
            bspline_degree_,
            control_points_.size(),
            knots_.size(),
            traj_u_start_,
            traj_u_end_);
    }
    // -------------------------
    // 发布 OffboardControlMode
    // -------------------------
    void publishOffboardControlMode()
    {
        px4_msgs::msg::OffboardControlMode msg{};

        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;

        msg.timestamp = timestamp();

        offboard_control_mode_publisher_->publish(msg);
    }

    // -------------------------
    // ENU / odom_world -> PX4 NED setpoint
    // 当前默认：
    //   odom_world x -> PX4 x
    //   odom_world y -> PX4 y
    //   odom_world z -> PX4 -z
    //
    // 如果你的 odom_world 是 x/y 交换后的，就把参数 swap_xy 设 true。
    // -------------------------
    std::array<float, 3> enuToPx4Position(
        double x,
        double y,
        double z)
    {
        if (swap_xy_)
        {
            return {
                static_cast<float>(y),
                static_cast<float>(x),
                static_cast<float>(-z)
            };
        }

        return {
            static_cast<float>(x),
            static_cast<float>(y),
            static_cast<float>(-z)
        };
    }

    // -------------------------
    // 发布 PX4 位置 setpoint
    // 参数输入为 ENU / odom_world 坐标，z 正向上
    // -------------------------
    void publishPositionSetpointENU(
        double x,
        double y,
        double z,
        float yaw = 0.0f)
    {
        px4_msgs::msg::TrajectorySetpoint msg{};

        msg.position = enuToPx4Position(x, y, z);
        msg.yaw = yaw;
        msg.timestamp = timestamp();

        trajectory_setpoint_publisher_->publish(msg);
    }

    // -------------------------
    // 距离判断，ENU
    // -------------------------
    double distanceTo(
        double x,
        double y,
        double z)
    {
        if (!has_odom_)
        {
            return 999.0;
        }

        double dx = odom_x_ - x;
        double dy = odom_y_ - y;
        double dz = odom_z_ - z;

        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    double distanceToGoal()
    {
        if (!has_odom_ || !has_goal_)
        {
            return 999.0;
        }

        double dx = odom_x_ - goal_x_;
        double dy = odom_y_ - goal_y_;
        double dz = odom_z_ - goal_z_;

        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    int findKnotSpan(double u)
    {
        int n = static_cast<int>(control_points_.size()) - 1;
        int p = bspline_degree_;

        if (u >= knots_[n + 1])
        {
            return n;
        }

        if (u <= knots_[p])
        {
            return p;
        }

        int low = p;
        int high = n + 1;
        int mid = (low + high) / 2;

        while (u < knots_[mid] || u >= knots_[mid + 1])
        {
            if (u < knots_[mid])
            {
                high = mid;
            }
            else
            {
                low = mid;
            }

            mid = (low + high) / 2;
        }

        return mid;
    }

    bool evaluateBSpline(
        double u,
        std::array<double, 3> &out)
    {
        int n = static_cast<int>(control_points_.size()) - 1;
        int p = bspline_degree_;

        if (control_points_.empty())
        {
            return false;
        }

        if (knots_.size() < static_cast<size_t>(n + p + 2))
        {
            RCLCPP_WARN(this->get_logger(), "Invalid knots size.");
            return false;
        }

        // 限制 u 不越界
        u = std::clamp(u, traj_u_start_, traj_u_end_);

        int k = findKnotSpan(u);

        std::vector<std::array<double, 3>> d;
        d.resize(p + 1);

        for (int j = 0; j <= p; ++j)
        {
            d[j] = control_points_[k - p + j];
        }

        // De Boor 算法
        for (int r = 1; r <= p; ++r)
        {
            for (int j = p; j >= r; --j)
            {
                int idx = k - p + j;

                double denom = knots_[idx + p + 1 - r] - knots_[idx];

                double alpha = 0.0;
                if (std::abs(denom) > 1e-6)
                {
                    alpha = (u - knots_[idx]) / denom;
                }

                d[j][0] = (1.0 - alpha) * d[j - 1][0] + alpha * d[j][0];
                d[j][1] = (1.0 - alpha) * d[j - 1][1] + alpha * d[j][1];
                d[j][2] = (1.0 - alpha) * d[j - 1][2] + alpha * d[j][2];
            }
        }

        out = d[p];
        return true;
    }

    // -------------------------
    // ARM / OFFBOARD
    // -------------------------
    void arm()
    {
        publishVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0);
    }

    void engageOffboardMode()
    {
        publishVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
            1,
            6);
    }

    void publishVehicleCommand(
        uint16_t command,
        float param1 = 0.0,
        float param2 = 0.0)
    {
        px4_msgs::msg::VehicleCommand msg{};

        msg.command = command;
        msg.param1 = param1;
        msg.param2 = param2;

        msg.target_system = 1;
        msg.target_component = 1;

        msg.source_system = 1;
        msg.source_component = 1;

        msg.from_external = true;

        msg.timestamp = timestamp();

        vehicle_command_publisher_->publish(msg);
    }

    // -------------------------
    // 主循环
    // -------------------------
    void timerCallback()
    {
        publishOffboardControlMode();

        // 默认先起飞/悬停到 takeoff 点
        if (state_ == ControlState::TAKEOFF_HOLD)
        {
            publishPositionSetpointENU(takeoff_x_, takeoff_y_, takeoff_z_, 0.0f);

            if (has_odom_ && distanceTo(takeoff_x_, takeoff_y_, takeoff_z_) < reach_thresh_)
            {
                hold_x_ = takeoff_x_;
                hold_y_ = takeoff_y_;
                hold_z_ = takeoff_z_;

                state_ = ControlState::WAIT_BSPLINE;

                RCLCPP_INFO(
                    this->get_logger(),
                    "Reached takeoff hold point. Waiting for bspline.");
            }
        }
        else if (state_ == ControlState::WAIT_BSPLINE)
        {
            // 没有轨迹时持续悬停
            publishPositionSetpointENU(hold_x_, hold_y_, hold_z_, 0.0f);
        }
        else if (state_ == ControlState::TRACK_BSPLINE)
        {
            if (!has_active_bspline_ ||
                control_points_.empty() ||
                knots_.empty())
            {
                if (has_odom_)
                {
                    hold_x_ = odom_x_;
                    hold_y_ = odom_y_;
                    hold_z_ = odom_z_;
                }

                state_ = ControlState::HOLD;

                RCLCPP_WARN(
                    this->get_logger(),
                    "No active bspline. Hold current position.");
            }
            else
            {
                double elapsed = (this->now() - traj_receive_time_).seconds();

                double u = traj_u_start_ + elapsed;

                if (u >= traj_u_end_)
                {
                    double dist_to_goal = distanceToGoal();

                    if (has_goal_ && dist_to_goal < 0.7)
                    {
                        hold_x_ = goal_x_;
                        hold_y_ = goal_y_;
                        hold_z_ = goal_z_;

                        RCLCPP_INFO(
                            this->get_logger(),
                            "Bspline finished near goal. Hold goal_pose: x=%.2f y=%.2f z=%.2f",
                            hold_x_,
                            hold_y_,
                            hold_z_);
                    }
                    else if (has_odom_)
                    {
                        hold_x_ = odom_x_;
                        hold_y_ = odom_y_;
                        hold_z_ = odom_z_;

                        RCLCPP_WARN(
                            this->get_logger(),
                            "Bspline finished but not near goal. Hold current odom: x=%.2f y=%.2f z=%.2f, dist_to_goal=%.2f",
                            hold_x_,
                            hold_y_,
                            hold_z_,
                            dist_to_goal);
                    }

                    has_active_bspline_ = false;
                    state_ = ControlState::HOLD;
                }
                else
                {
                    std::array<double, 3> target;

                    if (evaluateBSpline(u, target))
                    {
                        publishPositionSetpointENU(
                            target[0],
                            target[1],
                            target[2],
                            0.0f);
                    }
                    else
                    {
                        if (has_odom_)
                        {
                            hold_x_ = odom_x_;
                            hold_y_ = odom_y_;
                            hold_z_ = odom_z_;
                        }

                        has_active_bspline_ = false;
                        state_ = ControlState::HOLD;

                        RCLCPP_WARN(
                            this->get_logger(),
                            "Failed to evaluate bspline. Hold current position.");
                    }
                }
            }
        }
        else if (state_ == ControlState::HOLD)
        {
            publishPositionSetpointENU(hold_x_, hold_y_, hold_z_, 0.0f);
        }

        // 和 go/circle 一样，先发若干 setpoint，再切 Offboard + ARM
        if (auto_arm_ && offboard_setpoint_counter_ >= 10 && offboard_setpoint_counter_ < 80)
        {
            if (offboard_setpoint_counter_ % 10 == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Try switch to OFFBOARD and ARM");

                engageOffboardMode();
                arm();
            }
        }

        if (offboard_setpoint_counter_ < 100)
        {
            offboard_setpoint_counter_++;
        }    
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<BsplinePx4Bridge>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
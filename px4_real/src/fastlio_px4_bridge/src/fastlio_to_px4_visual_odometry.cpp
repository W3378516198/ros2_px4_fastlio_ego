#include <cmath>
#include <array>
#include <limits>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/timesync_status.hpp>

using std::placeholders::_1;

class FastlioToPx4VisualOdometry : public rclcpp::Node
{
public:
    FastlioToPx4VisualOdometry()
    : Node("fastlio_to_px4_visual_odometry")
    {
        declare_parameter<std::string>("input_odom_topic", "/Odometry");
        declare_parameter<bool>("use_timesync", true);
        declare_parameter<bool>("publish_velocity_from_delta", true);
        declare_parameter<bool>("publish_orientation", false);  
        declare_parameter<double>("pos_var_xy", 0.02);
        declare_parameter<double>("pos_var_z", 0.04);
        declare_parameter<double>("rot_var", 0.05);
        declare_parameter<double>("vel_var", 0.10);

        get_parameter("input_odom_topic", input_odom_topic_);
        get_parameter("use_timesync", use_timesync_);
        get_parameter("publish_velocity_from_delta", publish_velocity_from_delta_);
        get_parameter("publish_orientation", publish_orientation_);

        get_parameter("pos_var_xy", pos_var_xy_);
        get_parameter("pos_var_z", pos_var_z_);
        get_parameter("rot_var", rot_var_);
        get_parameter("vel_var", vel_var_);

        auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
        auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            input_odom_topic_,
            odom_qos,
            std::bind(&FastlioToPx4VisualOdometry::odomCallback, this, _1));

        visual_odom_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry",
            px4_qos);

        timesync_sub_ = create_subscription<px4_msgs::msg::TimesyncStatus>(
            "/fmu/out/timesync_status",
            px4_qos,
            std::bind(&FastlioToPx4VisualOdometry::timesyncCallback, this, _1));

        RCLCPP_WARN(
            get_logger(),
            "FAST-LIO odom bridge started. input=%s output=/fmu/in/vehicle_visual_odometry",
            input_odom_topic_.c_str());

        RCLCPP_WARN(
            get_logger(),
            "Frame conversion: FAST-LIO FLU/camera_init -> PX4 FRD local frame");
    }

private:
    struct QuatXYZW
    {
        double x;
        double y;
        double z;
        double w;
    };

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr visual_odom_pub_;

    std::string input_odom_topic_ = "/Odometry";

    bool use_timesync_ = true;
    bool has_timesync_ = false;
    int64_t timesync_offset_us_ = 0;

    bool publish_velocity_from_delta_ = true;
    bool publish_orientation_ = false;
    bool has_prev_ = false;
    rclcpp::Time prev_stamp_;
    std::array<float, 3> prev_pos_frd_{0.0f, 0.0f, 0.0f};

    double pos_var_xy_ = 0.02;
    double pos_var_z_ = 0.04;
    double rot_var_ = 0.05;
    double vel_var_ = 0.10;

    uint64_t nowPx4TimeUs()
    {
        const int64_t now_us =
            static_cast<int64_t>(this->get_clock()->now().nanoseconds() / 1000);

        if (use_timesync_ && has_timesync_)
        {
            return static_cast<uint64_t>(now_us + timesync_offset_us_);
        }

        return static_cast<uint64_t>(now_us);
    }

    uint64_t rosStampToPx4TimeUs(const builtin_interfaces::msg::Time &stamp)
    {
        const int64_t stamp_us =
            static_cast<int64_t>(stamp.sec) * 1000000LL +
            static_cast<int64_t>(stamp.nanosec) / 1000LL;

        if (stamp_us <= 0)
        {
            return nowPx4TimeUs();
        }

        if (use_timesync_ && has_timesync_)
        {
            return static_cast<uint64_t>(stamp_us + timesync_offset_us_);
        }

        return static_cast<uint64_t>(stamp_us);
    }

    void timesyncCallback(const px4_msgs::msg::TimesyncStatus::SharedPtr msg)
    {
        timesync_offset_us_ = msg->estimated_offset;
        has_timesync_ = true;
    }

    static QuatXYZW multiply(const QuatXYZW &a, const QuatXYZW &b)
    {
        QuatXYZW r;

        r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
        r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
        r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
        r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

        return r;
    }

    static QuatXYZW conjugate(const QuatXYZW &q)
    {
        return {-q.x, -q.y, -q.z, q.w};
    }

    static QuatXYZW normalize(const QuatXYZW &q)
    {
        const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);

        if (n < 1e-9)
        {
            return {0.0, 0.0, 0.0, 1.0};
        }

        return {q.x / n, q.y / n, q.z / n, q.w / n};
    }

    /*
     * FAST-LIO 常见输出：
     *   camera_init / body 近似 ROS FLU：
     *   x 前，y 左，z 上
     *
     * PX4 外部里程计这里使用 FRD：
     *   x 前，y 右，z 下
     *
     * 位置转换：
     *   x_frd =  x_flu
     *   y_frd = -y_flu
     *   z_frd = -z_flu
     */
    static std::array<float, 3> positionFluToFrd(double x, double y, double z)
    {
        return {
            static_cast<float>(x),
            static_cast<float>(-y),
            static_cast<float>(-z)
        };
    }

    /*
     * 姿态转换：
     * 源：body_FLU 在 world_FLU 下的姿态
     * 目标：body_FRD 在 world_FRD 下的姿态
     *
     * 两边都绕 x 轴翻转 180°：
     *   R_target = C * R_source * C^-1
     */
    static QuatXYZW quatFluToFrd(const geometry_msgs::msg::Quaternion &q_ros)
    {
        QuatXYZW q_src{
            q_ros.x,
            q_ros.y,
            q_ros.z,
            q_ros.w
        };

        q_src = normalize(q_src);

        const QuatXYZW q_flip_x{1.0, 0.0, 0.0, 0.0};
        const QuatXYZW q_target =
            multiply(multiply(q_flip_x, q_src), conjugate(q_flip_x));

        return normalize(q_target);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        const auto &p = odom->pose.pose.position;
        const auto &q = odom->pose.pose.orientation;

        if (!std::isfinite(p.x) ||
            !std::isfinite(p.y) ||
            !std::isfinite(p.z))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Received non-finite FAST-LIO position. Skip.");
            return;
        }

        if (publish_orientation_ &&
            (!std::isfinite(q.x) ||
            !std::isfinite(q.y) ||
            !std::isfinite(q.z) ||
            !std::isfinite(q.w)))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Received non-finite FAST-LIO orientation. Skip.");
            return;
        }
        px4_msgs::msg::VehicleOdometry msg{};

        msg.timestamp = nowPx4TimeUs();
        msg.timestamp_sample = rosStampToPx4TimeUs(odom->header.stamp);

        msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;

        const auto pos_frd = positionFluToFrd(p.x, p.y, p.z);
        msg.position = pos_frd;

        if (publish_orientation_)
        {
            const auto q_frd = quatFluToFrd(q);

            msg.q = {
                static_cast<float>(q_frd.w),
                static_cast<float>(q_frd.x),
                static_cast<float>(q_frd.y),
                static_cast<float>(q_frd.z)
            };

            msg.orientation_variance = {
                static_cast<float>(rot_var_),
                static_cast<float>(rot_var_),
                static_cast<float>(rot_var_)
            };
        }
        else
        {
            const float nan = std::numeric_limits<float>::quiet_NaN();

            msg.q = {nan, nan, nan, nan};
            msg.orientation_variance = {nan, nan, nan};
        }

        const float nan = std::numeric_limits<float>::quiet_NaN();

        msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD;

        if (publish_velocity_from_delta_ && has_prev_)
        {
            const rclcpp::Time current_stamp(odom->header.stamp);
            const double dt = (current_stamp - prev_stamp_).seconds();

            if (dt > 1e-3 && dt < 0.5)
            {
                msg.velocity = {
                    static_cast<float>((pos_frd[0] - prev_pos_frd_[0]) / dt),
                    static_cast<float>((pos_frd[1] - prev_pos_frd_[1]) / dt),
                    static_cast<float>((pos_frd[2] - prev_pos_frd_[2]) / dt)
                };

                msg.velocity_variance = {
                    static_cast<float>(vel_var_),
                    static_cast<float>(vel_var_),
                    static_cast<float>(vel_var_)
                };
            }
            else
            {
                msg.velocity = {nan, nan, nan};
                msg.velocity_variance = {nan, nan, nan};
            }
        }
        else
        {
            msg.velocity = {nan, nan, nan};
            msg.velocity_variance = {nan, nan, nan};
        }

        msg.angular_velocity = {nan, nan, nan};

        msg.position_variance = {
            static_cast<float>(pos_var_xy_),
            static_cast<float>(pos_var_xy_),
            static_cast<float>(pos_var_z_)
        };

        msg.reset_counter = 0;
        msg.quality = 100;

        visual_odom_pub_->publish(msg);

        prev_stamp_ = rclcpp::Time(odom->header.stamp);
        prev_pos_frd_ = pos_frd;
        has_prev_ = true;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FastlioToPx4VisualOdometry>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
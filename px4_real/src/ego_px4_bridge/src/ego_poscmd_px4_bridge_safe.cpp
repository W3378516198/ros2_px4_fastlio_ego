#include <chrono>
#include <cmath>
#include <array>
#include <limits>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/failsafe_flags.hpp>

#include <quadrotor_msgs/msg/position_command.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

/*
 * 作用：
 *   把 EGO Planner 发布的 pos_cmd 转成 PX4 能听懂的 TrajectorySetpoint。
 *
 * 和直接订阅 /planning/bspline 的区别：
 *   本节点不再自己计算 B 样条、不再自己推进 u、不再自己做 De Boor 采样。
 *   EGO Planner 已经把轨迹采样成 pos_cmd，本节点只负责：
 *     1. 接收 EGO 的 position / velocity / acceleration / yaw
 *     2. 做坐标系转换
 *     3. 持续发送 PX4 OffboardControlMode 和 TrajectorySetpoint
 *     4. 检查指令是否过期、PX4 状态是否正常、无人机是否明显跟不上目标点
 *
 * 注意：
 *   它不替代 FAST-LIO2 -> PX4 定位桥。
 *     fastlio_px4_bridge：告诉 PX4 “我现在在哪里”。
 *     ego_poscmd_px4_bridge_safe：告诉 PX4 “我要往哪里飞”。
 *
 * 真机首次带电测试必须拆桨。
 */
class EgoPoscmdPx4BridgeSafe : public rclcpp::Node
{
public:
    EgoPoscmdPx4BridgeSafe()
    : Node("ego_poscmd_px4_bridge_safe")
    {
        declare_parameter<std::string>("pos_cmd_topic", "/drone_0_planning/pos_cmd");
        declare_parameter<std::string>("vehicle_status_topic", "/fmu/out/vehicle_status_v1");
        declare_parameter<bool>("use_timesync", true);

        declare_parameter<bool>("use_velocity", false);
        declare_parameter<bool>("use_acceleration", false);
        declare_parameter<bool>("use_yaw", false);

        declare_parameter<bool>("auto_offboard", false);
        declare_parameter<bool>("auto_arm", false);

        declare_parameter<double>("cmd_timeout", 0.5);
        declare_parameter<double>("stale_disarm_time", 2.0);
        declare_parameter<bool>("auto_disarm_on_stale", true);
        declare_parameter<bool>("auto_disarm_on_failsafe", true);

        declare_parameter<bool>("flip_y", true);
        declare_parameter<bool>("flip_z", true);

        declare_parameter<int>("prestream_count", 40);

        declare_parameter<bool>("enable_tracking_error_check", true);
        declare_parameter<double>("max_tracking_error", 1.2);
        declare_parameter<double>("tracking_error_timeout", 0.8);
        declare_parameter<bool>("auto_disarm_on_tracking_error", false);
        declare_parameter<bool>("allow_recover_after_tracking_error", false);
        declare_parameter<double>("tracking_error_recover_threshold", 0.6);

        get_parameter("pos_cmd_topic", pos_cmd_topic_);
        get_parameter("vehicle_status_topic", vehicle_status_topic_);
        get_parameter("use_timesync", use_timesync_);
        get_parameter("use_velocity", use_velocity_);
        get_parameter("use_acceleration", use_acceleration_);
        get_parameter("use_yaw", use_yaw_);
        get_parameter("auto_offboard", auto_offboard_);
        get_parameter("auto_arm", auto_arm_);
        get_parameter("cmd_timeout", cmd_timeout_);
        get_parameter("stale_disarm_time", stale_disarm_time_);
        get_parameter("auto_disarm_on_stale", auto_disarm_on_stale_);
        get_parameter("auto_disarm_on_failsafe", auto_disarm_on_failsafe_);
        get_parameter("flip_y", flip_y_);
        get_parameter("flip_z", flip_z_);
        get_parameter("prestream_count", prestream_count_);
        get_parameter("enable_tracking_error_check", enable_tracking_error_check_);
        get_parameter("max_tracking_error", max_tracking_error_);
        get_parameter("tracking_error_timeout", tracking_error_timeout_);
        get_parameter("auto_disarm_on_tracking_error", auto_disarm_on_tracking_error_);
        get_parameter("allow_recover_after_tracking_error", allow_recover_after_tracking_error_);
        get_parameter("tracking_error_recover_threshold", tracking_error_recover_threshold_);

        auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        auto planner_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        pos_cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
            pos_cmd_topic_,
            planner_qos,
            std::bind(&EgoPoscmdPx4BridgeSafe::posCmdCallback, this, _1));

        timesync_sub_ = create_subscription<px4_msgs::msg::TimesyncStatus>(
            "/fmu/out/timesync_status",
            px4_qos,
            std::bind(&EgoPoscmdPx4BridgeSafe::timesyncCallback, this, _1));

        status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
            vehicle_status_topic_,
            px4_qos,
            std::bind(&EgoPoscmdPx4BridgeSafe::statusCallback, this, _1));

        local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position",
            px4_qos,
            std::bind(&EgoPoscmdPx4BridgeSafe::localPositionCallback, this, _1));

        failsafe_sub_ = create_subscription<px4_msgs::msg::FailsafeFlags>(
            "/fmu/out/failsafe_flags",
            px4_qos,
            std::bind(&EgoPoscmdPx4BridgeSafe::failsafeCallback, this, _1));

        offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode",
            px4_qos);

        trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint",
            px4_qos);

        vehicle_cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command",
            px4_qos);

        timer_ = create_wall_timer(
            50ms,
            std::bind(&EgoPoscmdPx4BridgeSafe::timerCallback, this));

        RCLCPP_WARN(
            get_logger(),
            "EGO pos_cmd -> PX4 bridge started. pos_cmd=%s, status=%s",
            pos_cmd_topic_.c_str(),
            vehicle_status_topic_.c_str());

        RCLCPP_WARN(
            get_logger(),
            "auto_offboard=%s auto_arm=%s. First motor tests must be propellers REMOVED.",
            auto_offboard_ ? "true" : "false",
            auto_arm_ ? "true" : "false");

        RCLCPP_WARN(
            get_logger(),
            "tracking_error_check=%s max_error=%.2f timeout=%.2f auto_disarm_on_tracking_error=%s",
            enable_tracking_error_check_ ? "true" : "false",
            max_tracking_error_,
            tracking_error_timeout_,
            auto_disarm_on_tracking_error_ ? "true" : "false");
    }

    void safeStop()
    {
        RCLCPP_WARN(get_logger(), "Node stopping. Send DISARM for safety.");
        for (int i = 0; i < 5; ++i)
        {
            disarm();
            rclcpp::sleep_for(50ms);
        }
    }

private:
    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_sub_;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::FailsafeFlags>::SharedPtr failsafe_sub_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::string pos_cmd_topic_ = "/drone_0_planning/pos_cmd";
    std::string vehicle_status_topic_ = "/fmu/out/vehicle_status_v1";

    bool use_timesync_ = true;
    bool use_velocity_ = false;
    bool use_acceleration_ = false;
    bool use_yaw_ = false;

    bool auto_offboard_ = false;
    bool auto_arm_ = false;
    bool auto_disarm_on_stale_ = true;
    bool auto_disarm_on_failsafe_ = true;

    bool flip_y_ = true;
    bool flip_z_ = true;

    double cmd_timeout_ = 0.5;
    double stale_disarm_time_ = 2.0;
    int prestream_count_ = 40;

    bool enable_tracking_error_check_ = true;
    double max_tracking_error_ = 1.2;
    double tracking_error_timeout_ = 0.8;
    bool auto_disarm_on_tracking_error_ = false;
    bool allow_recover_after_tracking_error_ = false;
    double tracking_error_recover_threshold_ = 0.6;

    bool has_timesync_ = false;
    int64_t timesync_offset_us_ = 0;

    bool has_cmd_ = false;
    rclcpp::Time last_cmd_time_;

    std::array<float, 3> target_pos_{0.0f, 0.0f, -1.0f};
    std::array<float, 3> target_vel_{
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN()
    };
    std::array<float, 3> target_acc_{
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN()
    };
    float target_yaw_ = std::numeric_limits<float>::quiet_NaN();

    std::array<float, 3> latest_cmd_pos_{0.0f, 0.0f, -1.0f};
    std::array<float, 3> latest_cmd_vel_{
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN()
    };
    std::array<float, 3> latest_cmd_acc_{
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN()
    };
    float latest_cmd_yaw_ = std::numeric_limits<float>::quiet_NaN();

    bool has_status_ = false;
    bool has_local_position_msg_ = false;
    bool has_failsafe_msg_ = false;

    bool preflight_ok_ = false;
    bool armed_ = false;
    bool local_position_ok_ = false;
    bool local_velocity_ok_ = false;
    bool failsafe_ok_ = true;

    float local_x_ = std::numeric_limits<float>::quiet_NaN();
    float local_y_ = std::numeric_limits<float>::quiet_NaN();
    float local_z_ = std::numeric_limits<float>::quiet_NaN();

    uint64_t setpoint_counter_ = 0;
    uint64_t mode_command_counter_ = 0;
    uint64_t arm_command_counter_ = 0;

    bool tracking_error_timer_started_ = false;
    rclcpp::Time tracking_error_start_time_;
    bool tracking_error_fault_ = false;

    static float nan()
    {
        return std::numeric_limits<float>::quiet_NaN();
    }

    static bool finite3(const std::array<float, 3> &v)
    {
        return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
    }

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

    void timesyncCallback(const px4_msgs::msg::TimesyncStatus::SharedPtr msg)
    {
        timesync_offset_us_ = msg->estimated_offset;
        has_timesync_ = true;
    }

    void statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        has_status_ = true;
        preflight_ok_ = msg->pre_flight_checks_pass;
        armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
    }

    void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        has_local_position_msg_ = true;

        local_position_ok_ = msg->xy_valid && msg->z_valid && !msg->dead_reckoning;
        local_velocity_ok_ = msg->v_xy_valid && msg->v_z_valid;

        if (std::isfinite(msg->x) && std::isfinite(msg->y) && std::isfinite(msg->z))
        {
            local_x_ = msg->x;
            local_y_ = msg->y;
            local_z_ = msg->z;
        }
    }

    void failsafeCallback(const px4_msgs::msg::FailsafeFlags::SharedPtr msg)
    {
        has_failsafe_msg_ = true;

        failsafe_ok_ =
            !msg->offboard_control_signal_lost &&
            !msg->local_position_invalid &&
            !msg->local_velocity_invalid &&
            !msg->fd_critical_failure;
    }

    std::array<float, 3> egoToPx4(double x, double y, double z) const
    {
        return {
            static_cast<float>(x),
            static_cast<float>(flip_y_ ? -y : y),
            static_cast<float>(flip_z_ ? -z : z)
        };
    }

    float egoYawToPx4(double yaw) const
    {
        return static_cast<float>(flip_y_ ? -yaw : yaw);
    }

    void posCmdCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
    {
        const auto &p = msg->position;
        const auto &v = msg->velocity;
        const auto &a = msg->acceleration;

        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Invalid EGO pos_cmd position. Skip.");
            return;
        }

        latest_cmd_pos_ = egoToPx4(p.x, p.y, p.z);

        if (use_velocity_ && std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z))
        {
            latest_cmd_vel_ = egoToPx4(v.x, v.y, v.z);
        }
        else
        {
            latest_cmd_vel_ = {nan(), nan(), nan()};
        }

        if (use_acceleration_ && std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z))
        {
            latest_cmd_acc_ = egoToPx4(a.x, a.y, a.z);
        }
        else
        {
            latest_cmd_acc_ = {nan(), nan(), nan()};
        }

        if (use_yaw_ && std::isfinite(msg->yaw))
        {
            latest_cmd_yaw_ = egoYawToPx4(msg->yaw);
        }
        else
        {
            latest_cmd_yaw_ = nan();
        }

        has_cmd_ = true;
        last_cmd_time_ = now();

        if (!tracking_error_fault_)
        {
            target_pos_ = latest_cmd_pos_;
            target_vel_ = latest_cmd_vel_;
            target_acc_ = latest_cmd_acc_;
            target_yaw_ = latest_cmd_yaw_;
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "EGO pos_cmd received. PX4 target pos[%.2f %.2f %.2f] vel_used=%s acc_used=%s tracking_fault=%s",
            latest_cmd_pos_[0],
            latest_cmd_pos_[1],
            latest_cmd_pos_[2],
            use_velocity_ ? "true" : "false",
            use_acceleration_ ? "true" : "false",
            tracking_error_fault_ ? "true" : "false");
    }

    bool hasFreshCommand() const
    {
        if (!has_cmd_)
        {
            return false;
        }
        return (now() - last_cmd_time_).seconds() < cmd_timeout_;
    }

    bool commandTooOldForFlight() const
    {
        if (!has_cmd_)
        {
            return true;
        }
        return (now() - last_cmd_time_).seconds() > stale_disarm_time_;
    }

    bool hasUsableLocalPosition() const
    {
        return has_local_position_msg_ &&
               local_position_ok_ &&
               std::isfinite(local_x_) &&
               std::isfinite(local_y_) &&
               std::isfinite(local_z_);
    }

    double trackingErrorToTarget(const std::array<float, 3> &target) const
    {
        if (!hasUsableLocalPosition() || !finite3(target))
        {
            return std::numeric_limits<double>::infinity();
        }

        const double dx = static_cast<double>(local_x_ - target[0]);
        const double dy = static_cast<double>(local_y_ - target[1]);
        const double dz = static_cast<double>(local_z_ - target[2]);

        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    void setHoldCurrentLocalPosition(const char *reason)
    {
        if (!hasUsableLocalPosition())
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Cannot hold current local position because PX4 local position is not usable. reason=%s",
                reason);
            return;
        }

        target_pos_ = {local_x_, local_y_, local_z_};
        target_vel_ = {nan(), nan(), nan()};
        target_acc_ = {nan(), nan(), nan()};
        target_yaw_ = nan();

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Hold current PX4 local position because %s. hold[%.2f %.2f %.2f]",
            reason,
            target_pos_[0],
            target_pos_[1],
            target_pos_[2]);
    }

    void checkTrackingError()
    {
        if (!enable_tracking_error_check_ || !hasUsableLocalPosition() || !has_cmd_)
        {
            return;
        }

        if (tracking_error_fault_)
        {
            setHoldCurrentLocalPosition("tracking error fault active");

            if (allow_recover_after_tracking_error_)
            {
                const double recover_err = trackingErrorToTarget(latest_cmd_pos_);
                if (recover_err < tracking_error_recover_threshold_)
                {
                    tracking_error_fault_ = false;
                    tracking_error_timer_started_ = false;

                    target_pos_ = latest_cmd_pos_;
                    target_vel_ = latest_cmd_vel_;
                    target_acc_ = latest_cmd_acc_;
                    target_yaw_ = latest_cmd_yaw_;

                    RCLCPP_WARN(
                        get_logger(),
                        "Tracking error recovered. err=%.2f < %.2f. Resume EGO pos_cmd.",
                        recover_err,
                        tracking_error_recover_threshold_);
                }
            }

            return;
        }

        const double err = trackingErrorToTarget(target_pos_);

        if (err <= max_tracking_error_)
        {
            tracking_error_timer_started_ = false;
            return;
        }

        if (!tracking_error_timer_started_)
        {
            tracking_error_timer_started_ = true;
            tracking_error_start_time_ = now();
        }

        const double over_time = (now() - tracking_error_start_time_).seconds();

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            500,
            "Tracking error high: err=%.2f m > %.2f m, duration=%.2f s / %.2f s",
            err,
            max_tracking_error_,
            over_time,
            tracking_error_timeout_);

        if (over_time >= tracking_error_timeout_)
        {
            tracking_error_fault_ = true;
            setHoldCurrentLocalPosition("tracking error too large");

            RCLCPP_ERROR(
                get_logger(),
                "Tracking error fault. err=%.2f m. Stop following EGO target and hold current position.",
                err);

            if (armed_ && auto_disarm_on_tracking_error_)
            {
                RCLCPP_ERROR(get_logger(), "Auto DISARM because tracking error fault.");
                disarm();
            }
        }
    }

    void publishOffboardControlMode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = nowPx4TimeUs();

        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;

        offboard_mode_pub_->publish(msg);
    }

    void publishTrajectorySetpoint()
    {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = nowPx4TimeUs();

        msg.position = target_pos_;
        msg.velocity = target_vel_;
        msg.acceleration = target_acc_;
        msg.yaw = target_yaw_;
        msg.yawspeed = nan();

        trajectory_pub_->publish(msg);
    }

    void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = nowPx4TimeUs();

        msg.command = command;
        msg.param1 = param1;
        msg.param2 = param2;

        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;

        vehicle_cmd_pub_->publish(msg);
    }

    void engageOffboardMode()
    {
        publishVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
            1.0f,
            6.0f);
    }

    void arm()
    {
        publishVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0f,
            0.0f);
    }

    void disarm()
    {
        publishVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0.0f,
            0.0f);
    }

    void timerCallback()
    {
        if (!has_cmd_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "No EGO pos_cmd received yet. Waiting.");
            return;
        }

        if (!hasFreshCommand())
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "EGO pos_cmd is stale. Hold current local position if possible.");

            setHoldCurrentLocalPosition("EGO pos_cmd stale");

            if (armed_ && auto_disarm_on_stale_ && commandTooOldForFlight())
            {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "EGO pos_cmd stale too long. DISARM.");
                disarm();
            }
        }
        else
        {
            checkTrackingError();
        }

        publishOffboardControlMode();
        publishTrajectorySetpoint();
        setpoint_counter_++;

        if (has_failsafe_msg_ && !failsafe_ok_)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "PX4 failsafe flag not OK. Hold and optionally DISARM.");

            setHoldCurrentLocalPosition("PX4 failsafe not OK");

            if (armed_ && auto_disarm_on_failsafe_)
            {
                disarm();
            }
            return;
        }

        if (!has_status_ || !has_local_position_msg_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Waiting PX4 status/local_position. has_status=%s has_local_pos=%s",
                has_status_ ? "true" : "false",
                has_local_position_msg_ ? "true" : "false");
            return;
        }

        if (!preflight_ok_ || !local_position_ok_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "PX4 not ready. preflight=%s local_pos=%s local_vel=%s",
                preflight_ok_ ? "true" : "false",
                local_position_ok_ ? "true" : "false",
                local_velocity_ok_ ? "true" : "false");
            return;
        }

        if (tracking_error_fault_)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Tracking error fault active. Do not auto Offboard/ARM. Restart node or allow recovery to resume.");
            return;
        }

        if (setpoint_counter_ < static_cast<uint64_t>(prestream_count_))
        {
            return;
        }

        if (auto_offboard_)
        {
            mode_command_counter_++;
            if (mode_command_counter_ % 20 == 1)
            {
                RCLCPP_INFO(get_logger(), "Try switch to OFFBOARD.");
                engageOffboardMode();
            }
        }

        if (auto_arm_)
        {
            arm_command_counter_++;
            if (arm_command_counter_ % 20 == 10)
            {
                RCLCPP_WARN(get_logger(), "Try ARM. Propellers must be removed for first tests.");
                arm();
            }
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<EgoPoscmdPx4BridgeSafe>();

    rclcpp::spin(node);

    node->safeStop();

    rclcpp::shutdown();

    return 0;
}
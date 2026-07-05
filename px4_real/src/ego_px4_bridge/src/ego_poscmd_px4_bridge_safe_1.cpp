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
 *   把 Ego Planner 发布的 pos_cmd 转成 PX4 能听懂的目标点。
 *
 * 它不替代 FAST-LIO2 -> PX4 定位桥。
 *   fastlio_px4_bridge：告诉 PX4 “我现在在哪里”。
 *   ego_poscmd_px4_bridge_safe：告诉 PX4 “我要往哪里飞”。
 *
 * 第一版默认只发送位置目标，不强行控制机头方向。
 * 所有带电机测试必须拆桨。
 */
class EgoPoscmdPx4BridgeSafe : public rclcpp::Node
{
public:
    EgoPoscmdPx4BridgeSafe()
    : Node("ego_poscmd_px4_bridge_safe")
    {
        // -------------------------
        // 可改参数
        // -------------------------
        declare_parameter<std::string>("pos_cmd_topic", "/drone_0_planning/pos_cmd");
        declare_parameter<std::string>("vehicle_status_topic", "/fmu/out/vehicle_status_v1");
        declare_parameter<bool>("use_timesync", true);

        // 第一版建议 false：先只发位置目标。位置目标调通后，再考虑打开速度/加速度。
        declare_parameter<bool>("use_velocity", false);
        declare_parameter<bool>("use_acceleration", false);
        declare_parameter<bool>("use_yaw", false);

        // 第一版建议 false：先只验证 setpoint，不自动切模式、不自动解锁。
        declare_parameter<bool>("auto_offboard", false);
        declare_parameter<bool>("auto_arm", false);

        // Ego Planner 指令多久没来，认为“规划器断了”。
        declare_parameter<double>("cmd_timeout", 0.5);

        // 指令短时间没来时，先保持最后一个目标点；超过 stale_disarm_time 后，如已解锁则自动上锁。
        declare_parameter<double>("stale_disarm_time", 2.0);
        declare_parameter<bool>("auto_disarm_on_stale", true);
        declare_parameter<bool>("auto_disarm_on_failsafe", true);

        // 坐标转换方式：默认和你的 FAST-LIO2 桥接保持一致：x 不变，y 取反，z 取反。
        // 通俗解释：Ego Planner 的“左/上”要转成 PX4 里的“右/下”。
        declare_parameter<bool>("flip_y", true);
        declare_parameter<bool>("flip_z", true);

        // 进入 Offboard / ARM 前，至少先连续发送一段目标点。
        declare_parameter<int>("prestream_count", 40);  // 20Hz 下大约 2 秒

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

        // 20Hz。PX4 要持续收到外部控制信号，不能只发一次。
        timer_ = create_wall_timer(
            50ms,
            std::bind(&EgoPoscmdPx4BridgeSafe::timerCallback, this));

        RCLCPP_WARN(
            get_logger(),
            "Ego pos_cmd -> PX4 bridge started. pos_cmd=%s, status=%s",
            pos_cmd_topic_.c_str(),
            vehicle_status_topic_.c_str());

        RCLCPP_WARN(
            get_logger(),
            "auto_offboard=%s auto_arm=%s. First tests must be propellers REMOVED.",
            auto_offboard_ ? "true" : "false",
            auto_arm_ ? "true" : "false");
    }

    // Ctrl+C 退出后，主函数会调用这个函数，尽量主动上锁。
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
    bool has_timesync_ = false;
    int64_t timesync_offset_us_ = 0;

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

    bool preflight_ok_ = false;
    bool armed_ = false;
    bool local_position_ok_ = false;
    bool local_velocity_ok_ = false;
    bool failsafe_ok_ = false;

    uint64_t setpoint_counter_ = 0;
    uint64_t mode_command_counter_ = 0;
    uint64_t arm_command_counter_ = 0;

    static float nan()
    {
        return std::numeric_limits<float>::quiet_NaN();
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
        preflight_ok_ = msg->pre_flight_checks_pass;
        armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
    }

    void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        local_position_ok_ = msg->xy_valid && msg->z_valid && !msg->dead_reckoning;
        local_velocity_ok_ = msg->v_xy_valid && msg->v_z_valid;
    }

    void failsafeCallback(const px4_msgs::msg::FailsafeFlags::SharedPtr msg)
    {
        failsafe_ok_ =
            !msg->offboard_control_signal_lost &&
            !msg->local_position_invalid &&
            !msg->local_velocity_invalid &&
            !msg->fd_critical_failure;
    }

    // 把 Ego Planner 的目标点换成 PX4 的目标点。
    // 默认：x 不变，y 取反，z 取反。
    std::array<float, 3> egoToPx4(double x, double y, double z)
    {
        return {
            static_cast<float>(x),
            static_cast<float>(flip_y_ ? -y : y),
            static_cast<float>(flip_z_ ? -z : z)
        };
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
                "Invalid pos_cmd position. Skip.");
            return;
        }

        target_pos_ = egoToPx4(p.x, p.y, p.z);

        if (use_velocity_ &&
            std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z))
        {
            target_vel_ = egoToPx4(v.x, v.y, v.z);
        }
        else
        {
            target_vel_ = {nan(), nan(), nan()};
        }

        if (use_acceleration_ &&
            std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z))
        {
            target_acc_ = egoToPx4(a.x, a.y, a.z);
        }
        else
        {
            target_acc_ = {nan(), nan(), nan()};
        }

        if (use_yaw_ && std::isfinite(msg->yaw))
        {
            target_yaw_ = static_cast<float>(msg->yaw);
        }
        else
        {
            // 第一版不强行控制机头方向。
            target_yaw_ = nan();
        }

        has_cmd_ = true;
        last_cmd_time_ = now();

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Ego pos_cmd -> PX4 target: pos[%.2f %.2f %.2f] vel_used=%s acc_used=%s",
            target_pos_[0],
            target_pos_[1],
            target_pos_[2],
            use_velocity_ ? "true" : "false",
            use_acceleration_ ? "true" : "false");
    }

    bool hasFreshCommand()
    {
        if (!has_cmd_)
        {
            return false;
        }
        return (now() - last_cmd_time_).seconds() < cmd_timeout_;
    }

    bool commandTooOldForFlight()
    {
        if (!has_cmd_)
        {
            return true;
        }
        return (now() - last_cmd_time_).seconds() > stale_disarm_time_;
    }

    void publishOffboardControlMode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = nowPx4TimeUs();

        // 这里表示：PX4 按“位置目标”飞。
        // 通俗说：我们告诉 PX4 “去这个点”，而不是直接告诉它每个电机怎么转。
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
                "No Ego pos_cmd received yet. Waiting.");
            return;
        }

        if (!hasFreshCommand())
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Ego pos_cmd is stale. Hold last target for a short time.");

            if (armed_ && auto_disarm_on_stale_ && commandTooOldForFlight())
            {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Ego pos_cmd stale too long. DISARM.");
                disarm();
            }
        }

        // 即使命令短暂变旧，也持续发送最后一个目标点，避免 PX4 立刻认为外部控制信号断了。
        publishOffboardControlMode();
        publishTrajectorySetpoint();
        setpoint_counter_++;

        if (!failsafe_ok_)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "PX4 failsafe flag not OK. local/offboard state may be bad.");
            if (armed_ && auto_disarm_on_failsafe_)
            {
                disarm();
            }
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

        // 先连续发一段目标点，再允许切 Offboard / ARM。
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
                RCLCPP_WARN(get_logger(), "Try ARM. Propellers must be removed.");
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
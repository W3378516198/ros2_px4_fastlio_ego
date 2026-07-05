#include <chrono>
#include <limits>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/timesync_status.hpp>

using namespace std::chrono_literals;

class VelocityOffboardTest : public rclcpp::Node
{
public:
    VelocityOffboardTest()
    : Node("velocity_offboard_test")
    {
        declare_parameter<double>("vx", 0.0);
        declare_parameter<double>("vy", 0.0);
        declare_parameter<double>("vz", 0.0);   // PX4 NED: z 正方向向下，负数表示向上
        declare_parameter<bool>("auto_offboard", false);
        declare_parameter<bool>("auto_arm", false);

        get_parameter("vx", vx_);
        get_parameter("vy", vy_);
        get_parameter("vz", vz_);
        get_parameter("auto_offboard", auto_offboard_);
        get_parameter("auto_arm", auto_arm_);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        offboard_pub_ =
            create_publisher<px4_msgs::msg::OffboardControlMode>(
                "/fmu/in/offboard_control_mode", qos);

        setpoint_pub_ =
            create_publisher<px4_msgs::msg::TrajectorySetpoint>(
                "/fmu/in/trajectory_setpoint", qos);

        command_pub_ =
            create_publisher<px4_msgs::msg::VehicleCommand>(
                "/fmu/in/vehicle_command", qos);

        timesync_sub_ =
            create_subscription<px4_msgs::msg::TimesyncStatus>(
                "/fmu/out/timesync_status",
                qos,
                std::bind(
                    &VelocityOffboardTest::timesyncCallback,
                    this,
                    std::placeholders::_1));

        timer_ =
            create_wall_timer(
                100ms,
                std::bind(&VelocityOffboardTest::timerCallback, this));

        RCLCPP_WARN(
            get_logger(),
            "Velocity test started. auto_offboard=%s, auto_arm=%s, vx=%.2f vy=%.2f vz=%.2f",
            auto_offboard_ ? "true" : "false",
            auto_arm_ ? "true" : "false",
            vx_, vy_, vz_);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;

    bool has_timesync_ = false;
    int64_t timesync_offset_us_ = 0;

    uint64_t counter_ = 0;

    double vx_ = 0.0;
    double vy_ = 0.0;
    double vz_ = 0.0;

    bool auto_offboard_ = false;
    bool auto_arm_ = false;

    uint64_t timestamp()
    {
        auto now_us = this->get_clock()->now().nanoseconds() / 1000;

        if (has_timesync_)
        {
            return static_cast<uint64_t>(now_us + timesync_offset_us_);
        }

        return static_cast<uint64_t>(now_us);
    }

    void timesyncCallback(
        const px4_msgs::msg::TimesyncStatus::SharedPtr msg)
    {
        timesync_offset_us_ = msg->estimated_offset;
        has_timesync_ = true;
    }

    void publishOffboardControlMode()
    {
        px4_msgs::msg::OffboardControlMode msg{};

        msg.position = false;
        msg.velocity = true;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;

        msg.timestamp = timestamp();

        offboard_pub_->publish(msg);
    }

    void publishVelocitySetpoint()
    {
        const float nan = std::numeric_limits<float>::quiet_NaN();

        px4_msgs::msg::TrajectorySetpoint msg{};

        msg.position = {nan, nan, nan};
        msg.velocity = {
            static_cast<float>(vx_),
            static_cast<float>(vy_),
            static_cast<float>(vz_)
        };
        msg.acceleration = {nan, nan, nan};
        msg.yaw = 0.0f;

        msg.timestamp = timestamp();

        setpoint_pub_->publish(msg);
    }

    void publishVehicleCommand(
        uint16_t command,
        float param1 = 0.0f,
        float param2 = 0.0f)
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

        command_pub_->publish(msg);
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

    void timerCallback()
    {
        publishOffboardControlMode();
        publishVelocitySetpoint();

        // 先连续发布一段 setpoint，再允许切 Offboard / ARM
        if (counter_ == 20)
        {
            if (auto_offboard_)
            {
                RCLCPP_WARN(get_logger(), "Trying to switch to OFFBOARD");
                engageOffboardMode();
            }

            if (auto_arm_)
            {
                RCLCPP_WARN(get_logger(), "Trying to ARM");
                arm();
            }
        }

        if (counter_ < 100)
        {
            counter_++;
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VelocityOffboardTest>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
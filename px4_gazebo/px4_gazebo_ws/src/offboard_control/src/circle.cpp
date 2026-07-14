#include <chrono>
#include <cmath> // 引入数学库以使用 sin 和 cos

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node
{
public:

    OffboardControl()
    : Node("offboard_control")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        offboard_control_mode_publisher_ =
            create_publisher<
                px4_msgs::msg::OffboardControlMode>(
                    "/fmu/in/offboard_control_mode",
                    qos);

        trajectory_setpoint_publisher_ =
            create_publisher<
                px4_msgs::msg::TrajectorySetpoint>(
                    "/fmu/in/trajectory_setpoint",
                    qos);

        vehicle_command_publisher_ =
            create_publisher<
                px4_msgs::msg::VehicleCommand>(
                    "/fmu/in/vehicle_command",
                    qos);

        timesync_status_subscriber_ =
            create_subscription<
                px4_msgs::msg::TimesyncStatus>(
                    "/fmu/out/timesync_status",
                    qos,
                    std::bind(
                        &OffboardControl::timesync_status_callback,
                        this,
                        std::placeholders::_1));

        // 100ms 触发一次，即控制频率为 10Hz
        timer_ =
            create_wall_timer(
                100ms,
                std::bind(
                    &OffboardControl::timer_callback,
                    this));

        offboard_setpoint_counter_ = 0;
    }

private:

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_status_subscriber_;

    uint64_t offboard_setpoint_counter_;

    bool has_timesync_ = false;
    int64_t timesync_offset_us_ = 0;

    // --- 新增：画圆轨迹所需的物理变量 ---
    double theta_ = 0.0;           // 当前角度 (弧度)
    double radius_ = 4.0;          // 盘旋半径 (米)
    double omega_ = 0.5;           // 角速度 (弧度/秒)，控制飞得有多快
    double dt_ = 0.1;              // 定时器周期 (0.1秒 = 100ms)
    // ---------------------------------

    uint64_t timestamp()
    {
        auto now_us = this->get_clock()->now().nanoseconds() / 1000;
        if (has_timesync_)
        {
            return static_cast<uint64_t>(now_us + timesync_offset_us_);
        }
        return static_cast<uint64_t>(now_us);
    }

    void timesync_status_callback(
        const px4_msgs::msg::TimesyncStatus::SharedPtr msg)
    {
        timesync_offset_us_ = msg->estimated_offset;
        has_timesync_ = true;
    }

    void publish_offboard_control_mode()
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

    void publish_trajectory_setpoint()
    {
        px4_msgs::msg::TrajectorySetpoint msg{};

        // 计算圆周运动当前的 X 和 Y 坐标
        float x = radius_ * std::cos(theta_);
        float y = radius_ * std::sin(theta_);
        float z = -2.0; // NED坐标系中，Z轴向下为正。-2.0 即为上方 2 米

        msg.position = {x, y, z};

        // 计算偏航角 (Yaw)，让机头始终朝向飞行路径的切线方向
        // 飞行方向是角度 theta + 90度(即 π/2)
        msg.yaw = theta_ + 1.570796f; 

        msg.timestamp = timestamp();

        trajectory_setpoint_publisher_->publish(msg);

        // 当成功切入 Offboard 模式后，才开始让角度随时间增加，使无人机动起来
        if (offboard_setpoint_counter_ >= 10)
        {
            theta_ += omega_ * dt_;
            
            // 防止角度变量无限增大导致溢出，保持在 0 ~ 2π 之间
            if (theta_ > 2.0 * 3.14159265f) {
                theta_ -= 2.0 * 3.14159265f;
            }
        }
    }

    void arm()
    {
        publish_vehicle_command(
            px4_msgs::msg::VehicleCommand::
                VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0);
    }

    void engage_offboard_mode()
    {
        publish_vehicle_command(
            px4_msgs::msg::VehicleCommand::
                VEHICLE_CMD_DO_SET_MODE,
            1,
            6);
    }

    void publish_vehicle_command(
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

    void timer_callback()
    {
        publish_offboard_control_mode();

        // 每次循环都会计算新的圆周位置并发布
        publish_trajectory_setpoint();

        if (offboard_setpoint_counter_ == 10)
        {
            RCLCPP_INFO(get_logger(), "Switch to OFFBOARD");
            engage_offboard_mode();

            RCLCPP_INFO(get_logger(), "ARM");
            arm();
        }

        if (offboard_setpoint_counter_ < 11)
        {
            offboard_setpoint_counter_++;
        }
    }
};

int main(int argc,char* argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<OffboardControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
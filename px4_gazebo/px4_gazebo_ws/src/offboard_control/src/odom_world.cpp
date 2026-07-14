#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp> // 引入点云头文件
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath> 

class PX4OdomTFBridge : public rclcpp::Node
{
public:
    PX4OdomTFBridge() : Node("px4_odom_tf_bridge")
    {
        // 默认初始化一个时间，防止雷达还没发数据时报错
        current_sim_time_ = this->get_clock()->now();

        // 1. PX4 输入订阅
        rclcpp::QoS sub_qos(rclcpp::KeepLast(10));
        sub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            sub_qos,
            std::bind(&PX4OdomTFBridge::callback, this, std::placeholders::_1)
        );

        // 2. 【新增】雷达点云订阅，用来偷它的纯正仿真时间戳
        // 如果你的雷达话题不是 /mid360s/points/points，请改成你实际的
        rclcpp::QoS lidar_qos(rclcpp::KeepLast(10));
        lidar_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/mid360s/points/points",
            lidar_qos,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                // 实时更新并锁死当前的 Gazebo 仿真时间
                this->current_sim_time_ = msg->header.stamp;
            }
        );

        // 3. odom 输出
        rclcpp::QoS pub_qos(rclcpp::KeepLast(10));
        pub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_world", pub_qos);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "PX4 Odom + TF Time-Synced Bridge started");
    }

private:
void callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        // rclcpp::Time px4_time(msg->timestamp * 1000ull); 
        rclcpp::Time px4_time = current_sim_time_;
        // ==========================================
        // 核心姿态转换计算: PX4 (FRD/NED) -> ROS (FLU/ENU)
        // ==========================================
        // 1. 提取 PX4 原始四元数 (注意 px4_msgs 里 q 的顺序是 [w, x, y, z])
        // tf2::Quaternion 构造函数的参数顺序是 (x, y, z, w)
        tf2::Quaternion q_px4(msg->q[1], msg->q[2], msg->q[3], msg->q[0]); 

        // 2. 将 PX4 四元数转为欧拉角
        double roll_px4, pitch_px4, yaw_px4;
        tf2::Matrix3x3(q_px4).getRPY(roll_px4, pitch_px4, yaw_px4);

        // 3. 执行严格的欧拉角映射公式
        // - Roll: PX4 与 ROS 均是向右侧倾为正
        double roll_ros = roll_px4;
        // - Pitch: PX4 抬头为正，ROS (FLU右手系) 抬头为负
        double pitch_ros = -pitch_px4;
        // - Yaw: ROS的0度(东) 等于 PX4的90度，且旋转方向相反
        double yaw_ros = M_PI_2 - yaw_px4;

        // 4. 将转换后的正确欧拉角转回 ROS 的四元数
        tf2::Quaternion q_ros;
        q_ros.setRPY(roll_ros, pitch_ros, yaw_ros);

        // ==========================================
        // 发布 Odometry
        // ==========================================
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = px4_time;
        odom.header.frame_id = "odom_world";
        odom.child_frame_id = "x500_0/base_link";

        // 位置依然是基础的向量映射: X_ros=E(Y_px4), Y_ros=N(X_px4), Z_ros=U(-Z_px4)
        odom.pose.pose.position.x = msg->position[1];
        odom.pose.pose.position.y = msg->position[0];
        odom.pose.pose.position.z = -msg->position[2];

        // 赋值计算好的新四元数
        odom.pose.pose.orientation.x = q_ros.x();
        odom.pose.pose.orientation.y = q_ros.y();
        odom.pose.pose.orientation.z = q_ros.z();
        odom.pose.pose.orientation.w = q_ros.w();

        // 速度也遵循位置的映射规则
        odom.twist.twist.linear.x = msg->velocity[1];
        odom.twist.twist.linear.y = msg->velocity[0];
        odom.twist.twist.linear.z = -msg->velocity[2];

        pub_->publish(odom);

        // ==========================================
        // 发布 TF
        // ==========================================
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = px4_time;
        tf.header.frame_id = "odom_world";
        tf.child_frame_id = "x500_0/base_link";

        tf.transform.translation.x = odom.pose.pose.position.x;
        tf.transform.translation.y = odom.pose.pose.position.y;
        tf.transform.translation.z = odom.pose.pose.position.z;

        tf.transform.rotation.w = odom.pose.pose.orientation.w;
        tf.transform.rotation.x = odom.pose.pose.orientation.x;
        tf.transform.rotation.y = odom.pose.pose.orientation.y;
        tf.transform.rotation.z = odom.pose.pose.orientation.z;

        tf_broadcaster_->sendTransform(tf);
    }    
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_; // 【新增】
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    rclcpp::Time current_sim_time_; // 【新增】缓存仿真时间的变量
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4OdomTFBridge>());
    rclcpp::shutdown();
    return 0;
}
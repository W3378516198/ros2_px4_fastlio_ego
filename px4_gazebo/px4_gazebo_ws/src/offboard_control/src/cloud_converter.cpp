#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <vector>
#include <cstring>
#include <cmath>
#include <string>

class DirectEgoCloudConverter : public rclcpp::Node
{
public:
    DirectEgoCloudConverter()
    : Node("cloud_converter_node")
    {
        this->declare_parameter<std::string>(
            "input_topic",
            "/mid360s/points/points");

        this->declare_parameter<std::string>(
            "output_topic",
            "/ego_points");

        this->declare_parameter<std::string>(
            "target_frame",
            "odom_world");

        this->declare_parameter<std::string>(
            "source_frame",
            "mid360_link");

        // 10Hz 雷达，一圈约 0.1s
        this->declare_parameter<double>(
            "scan_period",
            0.1);

        this->get_parameter("input_topic", input_topic_);
        this->get_parameter("output_topic", output_topic_);
        this->get_parameter("target_frame", target_frame_);
        this->get_parameter("source_frame", source_frame_);
        this->get_parameter("scan_period", scan_period_);

        tf_buffer_ =
            std::make_shared<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        sub_ =
            this->create_subscription<sensor_msgs::msg::PointCloud2>(
                input_topic_,
                rclcpp::SensorDataQoS(),
                std::bind(
                    &DirectEgoCloudConverter::cloudCallback,
                    this,
                    std::placeholders::_1));

        pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                output_topic_,
                rclcpp::QoS(10).reliable());

        RCLCPP_INFO(
            this->get_logger(),
            "Direct ego cloud converter started.");

        RCLCPP_INFO(
            this->get_logger(),
            "input_topic=%s, output_topic=%s, source_frame=%s, target_frame=%s",
            input_topic_.c_str(),
            output_topic_.c_str(),
            source_frame_.c_str(),
            target_frame_.c_str());
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string input_topic_;
    std::string output_topic_;
    std::string target_frame_;
    std::string source_frame_;

    double scan_period_ = 0.1;

    bool hasField(
        const sensor_msgs::msg::PointCloud2 &msg,
        const std::string &field_name)
    {
        for (const auto &field : msg.fields)
        {
            if (field.name == field_name)
            {
                return true;
            }
        }

        return false;
    }

    sensor_msgs::msg::PointCloud2 addTimeFieldIfNeeded(
        const sensor_msgs::msg::PointCloud2 &msg)
    {
        if (hasField(msg, "time"))
        {
            return msg;
        }

        sensor_msgs::msg::PointCloud2 new_msg = msg;

        sensor_msgs::msg::PointField time_field;
        time_field.name = "time";
        time_field.offset = new_msg.point_step;
        time_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        time_field.count = 1;

        uint32_t old_point_step = new_msg.point_step;
        uint32_t new_point_step = old_point_step + 4;

        new_msg.fields.push_back(time_field);
        new_msg.point_step = new_point_step;
        new_msg.row_step = new_msg.point_step * new_msg.width;

        uint32_t width = new_msg.width;
        uint32_t height = new_msg.height;
        uint32_t num_points = width * height;

        std::vector<uint8_t> new_data(
            num_points * new_msg.point_step);

        for (uint32_t row = 0; row < height; ++row)
        {
            for (uint32_t col = 0; col < width; ++col)
            {
                uint32_t i = row * width + col;

                uint32_t old_offset =
                    row * msg.row_step + col * old_point_step;

                uint32_t new_offset =
                    i * new_msg.point_step;

                std::memcpy(
                    &new_data[new_offset],
                    &msg.data[old_offset],
                    old_point_step);

                float relative_time = 0.0f;

                if (width > 1)
                {
                    relative_time =
                        (static_cast<float>(col) /
                         static_cast<float>(width - 1))
                        * static_cast<float>(scan_period_);
                }

                std::memcpy(
                    &new_data[new_offset + old_point_step],
                    &relative_time,
                    sizeof(float));
            }
        }

        new_msg.data = std::move(new_data);
        new_msg.is_dense = false;

        return new_msg;
    }

    void cloudCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 1. 先补 time 字段
        sensor_msgs::msg::PointCloud2 cloud_with_time =
            addTimeFieldIfNeeded(*msg);

        // 2. TF 转换到 odom_world
        sensor_msgs::msg::PointCloud2 transformed_cloud;

        try
        {
            geometry_msgs::msg::TransformStamped transformStamped =
                tf_buffer_->lookupTransform(
                    target_frame_,
                    source_frame_,
                    tf2::TimePointZero);

            tf2::doTransform(
                cloud_with_time,
                transformed_cloud,
                transformStamped);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "TF transform failed: %s",
                ex.what());

            return;
        }

        // 3. 转成 PointXYZI，清理无效点
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(transformed_cloud, cloud);

        pcl::PointCloud<pcl::PointXYZI> clean_cloud;
        clean_cloud.points.reserve(cloud.points.size());

        for (const auto &pt : cloud.points)
        {
            if (!std::isfinite(pt.x) ||
                !std::isfinite(pt.y) ||
                !std::isfinite(pt.z))
            {
                continue;
            }

            pcl::PointXYZI p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;

            // EGO Planner 不依赖 intensity，统一给 1.0 即可
            p.intensity = 1.0f;

            clean_cloud.points.push_back(p);
        }

        clean_cloud.width =
            static_cast<uint32_t>(clean_cloud.points.size());

        clean_cloud.height = 1;
        clean_cloud.is_dense = true;

        // 4. 发布 /ego_points
        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(clean_cloud, out_msg);

        out_msg.header.stamp =
            transformed_cloud.header.stamp;

        out_msg.header.frame_id =
            target_frame_;

        pub_->publish(out_msg);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Published /ego_points directly from raw cloud. points=%u",
            clean_cloud.width);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<DirectEgoCloudConverter>());

    rclcpp::shutdown();

    return 0;
}
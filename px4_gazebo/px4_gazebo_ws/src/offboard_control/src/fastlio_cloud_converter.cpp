#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <limits>

class FastlioCloudConverter : public rclcpp::Node
{
public:
    FastlioCloudConverter()
    : Node("fastlio_cloud_converter")
    {
        this->declare_parameter<std::string>(
            "input_topic",
            "/mid360s/points/points");

        this->declare_parameter<std::string>(
            "output_topic",
            "/fastlio_points");

        this->declare_parameter<std::string>(
            "frame_id",
            "mid360_link");

        // 10Hz 雷达，一圈 0.1s；如果你的雷达是 15Hz，改成 1.0 / 15.0
        this->declare_parameter<double>(
            "scan_period",
            0.1);

        this->declare_parameter<int>(
            "default_ring",
            0);

        this->get_parameter("input_topic", input_topic_);
        this->get_parameter("output_topic", output_topic_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("scan_period", scan_period_);
        this->get_parameter("default_ring", default_ring_);

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(
                &FastlioCloudConverter::cloudCallback,
                this,
                std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_,
            rclcpp::SensorDataQoS());

        RCLCPP_INFO(
            this->get_logger(),
            "FAST-LIO cloud converter started.");

        RCLCPP_INFO(
            this->get_logger(),
            "input_topic=%s, output_topic=%s, frame_id=%s, scan_period=%.6f",
            input_topic_.c_str(),
            output_topic_.c_str(),
            frame_id_.c_str(),
            scan_period_);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    std::string input_topic_;
    std::string output_topic_;
    std::string frame_id_;

    double scan_period_ = 0.1;
    int default_ring_ = 0;

private:
    int getFieldIndex(
        const sensor_msgs::msg::PointCloud2 &msg,
        const std::string &name) const
    {
        for (size_t i = 0; i < msg.fields.size(); ++i)
        {
            if (msg.fields[i].name == name)
            {
                return static_cast<int>(i);
            }
        }

        return -1;
    }

    bool hasField(
        const sensor_msgs::msg::PointCloud2 &msg,
        const std::string &name) const
    {
        return getFieldIndex(msg, name) >= 0;
    }

    double readFieldAsDouble(
        const sensor_msgs::msg::PointCloud2 &msg,
        uint32_t point_offset,
        const std::string &field_name,
        double default_value = 0.0) const
    {
        int idx = getFieldIndex(msg, field_name);

        if (idx < 0)
        {
            return default_value;
        }

        const auto &field = msg.fields[idx];

        const uint8_t *ptr =
            msg.data.data() + point_offset + field.offset;

        switch (field.datatype)
        {
            case sensor_msgs::msg::PointField::INT8:
            {
                int8_t v;
                std::memcpy(&v, ptr, sizeof(v));
                return static_cast<double>(v);
            }

            case sensor_msgs::msg::PointField::UINT8:
            {
                uint8_t v;
                std::memcpy(&v, ptr, sizeof(v));
                return static_cast<double>(v);
            }

            case sensor_msgs::msg::PointField::INT16:
            {
                int16_t v;
                std::memcpy(&v, ptr, sizeof(v));
                return static_cast<double>(v);
            }

            case sensor_msgs::msg::PointField::UINT16:
            {
                uint16_t v;
                std::memcpy(&v, ptr, sizeof(v));
                return static_cast<double>(v);
            }

            case sensor_msgs::msg::PointField::INT32:
            {
                int32_t v;
                std::memcpy(&v, ptr, sizeof(v));
                return static_cast<double>(v);
            }

            case sensor_msgs::msg::PointField::UINT32:
            {
                uint32_t v;
                std::memcpy(&v, ptr, sizeof(v));
                return static_cast<double>(v);
            }

            case sensor_msgs::msg::PointField::FLOAT32:
            {
                float v;
                std::memcpy(&v, ptr, sizeof(v));
                return static_cast<double>(v);
            }

            case sensor_msgs::msg::PointField::FLOAT64:
            {
                double v;
                std::memcpy(&v, ptr, sizeof(v));
                return v;
            }

            default:
                return default_value;
        }
    }

    float calcRelativeTime(
        const sensor_msgs::msg::PointCloud2 &msg,
        uint32_t row,
        uint32_t col,
        uint32_t point_offset) const
    {
        // 如果原始点云已经有 time 字段，优先使用原始 time
        if (hasField(msg, "time"))
        {
            return static_cast<float>(
                readFieldAsDouble(msg, point_offset, "time", 0.0));
        }

        // 有些驱动叫 timestamp
        if (hasField(msg, "timestamp"))
        {
            return static_cast<float>(
                readFieldAsDouble(msg, point_offset, "timestamp", 0.0));
        }

        // 有些点云字段叫 offset_time，通常单位可能是 ns
        if (hasField(msg, "offset_time"))
        {
            double offset_time =
                readFieldAsDouble(msg, point_offset, "offset_time", 0.0);

            // 如果数值很大，按纳秒转秒处理
            if (offset_time > 1.0)
            {
                return static_cast<float>(offset_time * 1e-9);
            }

            return static_cast<float>(offset_time);
        }

        // 否则根据点在一帧中的列位置估算相对时间
        if (msg.width > 1)
        {
            return static_cast<float>(
                (static_cast<double>(col) /
                 static_cast<double>(msg.width - 1)) *
                scan_period_);
        }

        return 0.0f;
    }

    uint16_t calcRingOrLine(
        const sensor_msgs::msg::PointCloud2 &msg,
        uint32_t row,
        uint32_t point_offset) const
    {
        if (hasField(msg, "ring"))
        {
            return static_cast<uint16_t>(
                readFieldAsDouble(msg, point_offset, "ring", default_ring_));
        }

        if (hasField(msg, "line"))
        {
            return static_cast<uint16_t>(
                readFieldAsDouble(msg, point_offset, "line", default_ring_));
        }

        // 对于 organized cloud，可以用 row 近似作为线号
        if (msg.height > 1)
        {
            return static_cast<uint16_t>(row);
        }

        return static_cast<uint16_t>(default_ring_);
    }

    void addField(
        sensor_msgs::msg::PointCloud2 &msg,
        const std::string &name,
        uint32_t offset,
        uint8_t datatype,
        uint32_t count) const
    {
        sensor_msgs::msg::PointField field;
        field.name = name;
        field.offset = offset;
        field.datatype = datatype;
        field.count = count;

        msg.fields.push_back(field);
    }

    void cloudCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (msg->is_bigendian)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Big-endian PointCloud2 is not supported by this converter.");

            return;
        }

        if (!hasField(*msg, "x") ||
            !hasField(*msg, "y") ||
            !hasField(*msg, "z"))
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Input cloud has no x/y/z fields.");

            return;
        }

        sensor_msgs::msg::PointCloud2 out;

        out.header = msg->header;

        // 重点：FAST-LIO2 输入应保持雷达坐标系，不要转 odom_world
        out.header.frame_id = frame_id_;

        out.height = 1;
        out.is_bigendian = false;
        out.is_dense = false;

        /*
         * 输出字段：
         * x           float32
         * y           float32
         * z           float32
         * intensity   float32
         * time        float32，单位：秒，一帧内相对时间
         * offset_time uint32，单位：纳秒，一帧内相对时间
         * ring        uint16
         * line        uint16
         * tag         uint8
         *
         * 多放几个字段的原因：
         * 不同 FAST-LIO2 / FAST-LIO-ROS2 版本字段名不完全一致。
         * 多余字段通常不会影响读取，缺字段反而容易报错。
         */
        addField(out, "x",           0,  sensor_msgs::msg::PointField::FLOAT32, 1);
        addField(out, "y",           4,  sensor_msgs::msg::PointField::FLOAT32, 1);
        addField(out, "z",           8,  sensor_msgs::msg::PointField::FLOAT32, 1);
        addField(out, "intensity",   12, sensor_msgs::msg::PointField::FLOAT32, 1);
        addField(out, "time",        16, sensor_msgs::msg::PointField::FLOAT32, 1);
        addField(out, "offset_time", 20, sensor_msgs::msg::PointField::UINT32,  1);
        addField(out, "ring",        24, sensor_msgs::msg::PointField::UINT16,  1);
        addField(out, "line",        26, sensor_msgs::msg::PointField::UINT8,   1);
        addField(out, "tag",         27, sensor_msgs::msg::PointField::UINT8,   1);
        // 32 字节对齐，后面 3 字节作为 padding
        out.point_step = 32;

        const uint32_t input_width = msg->width;
        const uint32_t input_height = msg->height;
        const uint32_t input_points = input_width * input_height;

        out.data.reserve(input_points * out.point_step);

        uint32_t valid_points = 0;

        for (uint32_t row = 0; row < input_height; ++row)
        {
            for (uint32_t col = 0; col < input_width; ++col)
            {
                const uint32_t input_offset =
                    row * msg->row_step + col * msg->point_step;

                float x = static_cast<float>(
                    readFieldAsDouble(*msg, input_offset, "x",
                                      std::numeric_limits<float>::quiet_NaN()));

                float y = static_cast<float>(
                    readFieldAsDouble(*msg, input_offset, "y",
                                      std::numeric_limits<float>::quiet_NaN()));

                float z = static_cast<float>(
                    readFieldAsDouble(*msg, input_offset, "z",
                                      std::numeric_limits<float>::quiet_NaN()));

                if (!std::isfinite(x) ||
                    !std::isfinite(y) ||
                    !std::isfinite(z))
                {
                    continue;
                }

                float intensity = 1.0f;

                if (hasField(*msg, "intensity"))
                {
                    intensity = static_cast<float>(
                        readFieldAsDouble(*msg, input_offset, "intensity", 1.0));
                }
                else if (hasField(*msg, "reflectivity"))
                {
                    intensity = static_cast<float>(
                        readFieldAsDouble(*msg, input_offset, "reflectivity", 1.0));
                }

                float relative_time =
                    calcRelativeTime(*msg, row, col, input_offset);

                if (relative_time < 0.0f)
                {
                    relative_time = 0.0f;
                }

                uint32_t offset_time_ns =
                    static_cast<uint32_t>(
                        relative_time * 1e9);

                uint16_t ring =
                    calcRingOrLine(*msg, row, input_offset);

                uint8_t line = static_cast<uint8_t>(ring & 0xFF);

                if (hasField(*msg, "line"))
                {
                    line = static_cast<uint8_t>(
                        readFieldAsDouble(*msg, input_offset, "line", line));
                }

                uint8_t tag = 0;

                if (hasField(*msg, "tag"))
                {
                    tag = static_cast<uint8_t>(
                        readFieldAsDouble(*msg, input_offset, "tag", 0));
                }
                const size_t old_size = out.data.size();

                out.data.resize(old_size + out.point_step, 0);

                uint8_t *ptr = out.data.data() + old_size;

                std::memcpy(ptr + 0,  &x,              sizeof(float));
                std::memcpy(ptr + 4,  &y,              sizeof(float));
                std::memcpy(ptr + 8,  &z,              sizeof(float));
                std::memcpy(ptr + 12, &intensity,      sizeof(float));
                std::memcpy(ptr + 16, &relative_time,  sizeof(float));
                std::memcpy(ptr + 20, &offset_time_ns, sizeof(uint32_t));
                std::memcpy(ptr + 24, &ring,           sizeof(uint16_t));
                std::memcpy(ptr + 26, &line,           sizeof(uint8_t));
                std::memcpy(ptr + 27, &tag,            sizeof(uint8_t));
                ++valid_points;
            }
        }

        out.width = valid_points;
        out.row_step = out.width * out.point_step;

        pub_->publish(out);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Published FAST-LIO cloud: input=%u, output=%u, frame_id=%s",
            input_points,
            valid_points,
            out.header.frame_id.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<FastlioCloudConverter>());

    rclcpp::shutdown();

    return 0;
}
#include "nodes/lidar.hpp"
#include <cmath>
#include <numeric>
#include <limits>

namespace nodes {

    LidarFiltrResults LidarFiltr::apply_filter(const std::vector<float>& points, float angle_start, float angle_end, float range_min, float range_max) {
        std::vector<float> front{}, back{}, left{}, right{};
        constexpr float PI = 3.14159265f;
        constexpr float angle_range = PI / 8.0f;
        float angle_offset = PI / 4.0f;
        float angle_step = (angle_end - angle_start) / points.size();

        for (size_t i = 0; i < points.size(); ++i) {
            float angle = angle_start + i * angle_step;
            float distance = points[i];

            if (!std::isfinite(distance) || distance <= 0.0f) continue;
            std::clamp(distance, range_min, range_max);

            while (angle > PI) angle -= 2 * PI;
            while (angle < -PI) angle += 2 * PI;

            if (std::abs(angle) <= angle_range) {
                back.push_back(distance);
            } else if (std::abs(angle - PI) <= angle_range || std::abs(angle + PI) <= angle_range) {
                front.push_back(distance);
            } else if (angle > angle_range + angle_offset && angle < PI - angle_range + angle_offset) {
                right.push_back(distance);
            } else if (angle < -angle_range - angle_offset && angle > -PI + angle_range - angle_offset) {
                left.push_back(distance);
            }
        }

        auto average = [](const std::vector<float>& values) -> float {
            if (values.empty()) return std::numeric_limits<float>::quiet_NaN();
            return std::accumulate(values.begin(), values.end(), 0.0f) / values.size();
        };

        return LidarFiltrResults{
            .front = average(front),
            .back = average(back),
            .left = average(left),
            .right = average(right)
        };
    }

    LidarFilterNode::LidarFilterNode()
        : Node("lidar_filter")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/bpc_prp_robot/lidar", 10,
            std::bind(&LidarFilterNode::scan_callback, this, std::placeholders::_1)
        );
    }

    void LidarFilterNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        LidarFiltr filter;
        auto result = filter.apply_filter(msg->ranges, msg->angle_min, msg->angle_max, msg->range_min, msg->range_max);

        RCLCPP_INFO(this->get_logger(), "LIDAR Data - Front: %.2f, Back: %.2f, Left: %.2f, Right: %.2f",
                    result.front, result.back, result.left, result.right);
    }

}





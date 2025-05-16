#include "nodes/lidar.hpp"
#include <cmath>
#include <numeric>
#include <limits>

namespace nodes
{
    LidarFiltrResults LidarFiltr::apply_filter(const std::vector<float>& points, float angle_start, float angle_end,
                                               float range_min, float range_max)
    {
        std::vector<float> front{}, back{}, front_left{}, front_right{}, back_left{}, back_right{}, right_side{}, left_side{},
                          left_side_follow_front{}, left_side_follow_back{}, right_side_follow_front{}, right_side_follow_back{}, ffront_right{}, ffront_left{};

        constexpr float PI = 3.14159265f;
        constexpr float angle_range = PI / 6.0f;
        float angle_offset = PI / 4.0f;
        float angle_step = (angle_end - angle_start) / points.size();

        for (size_t i = 0; i < points.size(); ++i)
        {
            float angle = angle_start + i * angle_step;
            float distance = points[i];

            if (!std::isfinite(distance) || distance <= 0.0f) continue;
            std::clamp(distance, range_min, range_max);

            while (angle > PI) angle -= 2 * PI;
            while (angle < -PI) angle += 2 * PI;

            if (std::abs(angle) <= angle_range)
            {
                back.push_back(distance);
            }
            else if (std::abs(angle - PI) <= angle_range || std::abs(angle + PI) <= angle_range)
            {
                front.push_back(distance);
            }
            else if (angle > angle_range + angle_offset && angle < PI - angle_range + angle_offset)
            {
                //} else if (angle > angle_range + angle_offset && angle < PI - (angle_range + angle_offset)) {
                //} else if (angle > 3*PI/16 && angle < 8*PI/16) {
                back_right.push_back(distance);

                if (angle > (10 * PI / 16) && angle < (11 * PI / 16))
                {
                    front_right.push_back(distance);
                }
                if (angle > (5 * PI / 12) && angle < (7 * PI / 12))
                {
                    right_side.push_back(distance);
                }
                if (angle > (13 * PI / 24) && angle < (15 * PI / 24))
                {
                    right_side_follow_front.push_back(distance);
                }
                if (angle > (9 * PI / 24) && angle < (11 * PI / 24))
                {
                    right_side_follow_back.push_back(distance);
                }
            }
            else if (angle < -angle_range - angle_offset && angle > -PI + angle_range - angle_offset)
            {
                //} else if (angle < -angle_range - angle_offset && angle > -PI + (angle_range - angle_offset)) {
                //} else if (angle < -3*PI/16 && angle > -8*PI/16) {
                back_left.push_back(distance);

                if (angle < -(10 * PI / 16) && angle > -(11 * PI / 16))
                {
                    front_left.push_back(distance);
                }
                if (angle < -(5 * PI / 12) && angle > -(7 * PI / 12))
                {
                    left_side.push_back(distance);
                }
                if (angle < -(13 * PI / 24) && angle > -(15 * PI / 24))
                {
                    left_side_follow_front.push_back(distance);
                }
                if (angle < -(9 * PI / 24) && angle > -(11 * PI / 24))
                {
                    left_side_follow_back.push_back(distance);
                }
            }
        }

        auto average = [](const std::vector<float>& values) -> float
        {
            if (values.empty()) return std::numeric_limits<float>::quiet_NaN();
            return std::accumulate(values.begin(), values.end(), 0.0f) / values.size();
        };

        return LidarFiltrResults{
            .front = average(front),
            .back = average(back),
            .front_left = average(front_left),
            .front_right = average(front_right),
            .back_left = average(back_left),
            .back_right = average(back_right),
            .right_side = average(right_side),
            .left_side = average(left_side),
            .left_side_follow_front = average(left_side_follow_front),
            .left_side_follow_back = average(left_side_follow_back),
            .right_side_follow_front = average(right_side_follow_front),
            .right_side_follow_back = average(right_side_follow_back)
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

    void LidarFilterNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        LidarFiltr filter;
        auto result = filter.apply_filter(msg->ranges, msg->angle_min, msg->angle_max, msg->range_min, msg->range_max);

        RCLCPP_INFO(this->get_logger(), "LIDAR Data - Front: %.2f, Back: %.2f, Front_Left: %.2f, Front_Right: %.2f",
                    result.front, result.back, result.front_left, result.front_right);
    }
}

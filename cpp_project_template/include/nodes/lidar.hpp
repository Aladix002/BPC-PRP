#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

namespace nodes {

    struct LidarFiltrResults {
        float front;
        float back;
        float front_left;
        float front_right;
        float back_left;
        float back_right;
        float right_side;
        float left_side;
        float left_side_follow_front;
        float left_side_follow_back;
        float right_side_follow_front;
        float right_side_follow_back;
        float ffront_right;
        float ffront_left;

    };

    class LidarFiltr {
    public:
        LidarFiltrResults apply_filter(const std::vector<float>& points, float angle_start, float angle_end, float range_min, float range_max);
    };

    class LidarFilterNode : public rclcpp::Node {
    public:
        LidarFilterNode();

    private:
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    };

}

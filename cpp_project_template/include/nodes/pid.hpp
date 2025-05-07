#ifndef PID_H
#define PID_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>  // For line sensor callback
#include <memory>
#include "nodes/lidar.hpp"
#include "nodes/motor.hpp"
#include "nodes/imu_node.hpp"

namespace nodes {

    enum class DriveState {
        DRIVE_FORWARD,
        TURNING
    };

    class PidNode : public rclcpp::Node {
    public:
        PidNode(std::shared_ptr<ImuNode> imu_node);

    private:
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void left_line_callback(const std_msgs::msg::Bool::SharedPtr msg);   // Declaration
        void right_line_callback(const std_msgs::msg::Bool::SharedPtr msg);  // Declaration

        float Kp, Kd, Ki;
        float base_speed;
        float last_error;
        float integral_;

        DriveState state_;
        int turn_direction_; // -1 = doprava, 1 = doľava
        float turn_start_yaw_;

        bool left_line_detected_;
        bool right_line_detected_;

        std::shared_ptr<ImuNode> imu_node_;
        std::shared_ptr<MotorController> motor_controller_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_line_sub_;  // Line sensor subscriptions
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_line_sub_; // Line sensor subscriptions
    };

}

#endif // PID_H




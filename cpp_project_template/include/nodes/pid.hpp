#ifndef PID_H
#define PID_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <memory>
#include "nodes/lidar.hpp"
#include "nodes/motor.hpp"
#include "nodes/imu_node.hpp"
#include "std_msgs/msg/int32.hpp"

namespace nodes {

    enum class DriveState {
        DRIVE_FORWARD,
        TURNING
    };

    enum class ArucoTag {
        NONE = -1,
        ESCAPE_STRAIGHT = 0,
        ESCAPE_LEFT = 1,
        ESCAPE_RIGHT = 2,
        TREASURE_STRAIGHT = 10,
        TREASURE_LEFT = 11,
        TREASURE_RIGHT = 12
    };

    class PidNode : public rclcpp::Node {
    public:
        PidNode(std::shared_ptr<ImuNode> imu_node);

    private:
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void left_line_callback(const std_msgs::msg::Bool::SharedPtr msg);
        void right_line_callback(const std_msgs::msg::Bool::SharedPtr msg);
        void aruco_callback(const std_msgs::msg::Int32::SharedPtr msg);

        float Kp, Kd, Ki;
        float base_speed;
        float last_error;
        float integral_;

        DriveState state_;
        int turn_direction_; // -1 = doprava, 1 = doľava
        float turn_start_yaw_;

        std::shared_ptr<ImuNode> imu_node_;
        std::shared_ptr<MotorController> motor_controller_;

        // Subscriptions
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_line_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_line_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr aruco_sub_;

        // Last seen ArUco tag
        int last_aruco_id_ = -1;
        rclcpp::Time last_aruco_time_;
    };

}

#endif // PID_H






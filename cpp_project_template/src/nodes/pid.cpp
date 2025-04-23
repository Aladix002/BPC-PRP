#include "nodes/pid.hpp"
#include <algorithm>
#include <cmath>

namespace nodes {

    constexpr float PI = 3.14159265f;

    PidNode::PidNode(std::shared_ptr<ImuNode> imu_node)
        : Node("pid_node"),
          Kp(5), Kd(6), Ki(0),
          base_speed(136.0f),
          last_error(0.0f), integral_(0.0f),
          imu_node_(imu_node),
          state_(DriveState::DRIVE_FORWARD),
          turn_direction_(0),
          turn_start_yaw_(0.0f)
    {
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/bpc_prp_robot/lidar", 10,
            std::bind(&PidNode::scan_callback, this, std::placeholders::_1)
        );

        motor_controller_ = std::make_shared<MotorController>();
    }

    void PidNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        LidarFiltr filter;
        auto result = filter.apply_filter(msg->ranges, msg->angle_min, msg->angle_max, msg->range_min, msg->range_max);

        float front = result.front;
        float left = result.left;
        float right = result.right;
        float side_threshold=0.5f;
        float left_motor_speed=0, right_motor_speed=0;
        float front_treshold = 0.3f;

        if (state_ == DriveState::DRIVE_FORWARD) {
            if (front < front_treshold) {
                // Zastavíme pred otočkou
                motor_controller_->set_motor_speeds({128.0f, 128.0f});

                // Pripravíme otočku
                state_ = DriveState::TURNING;
                turn_start_yaw_ = imu_node_->getIntegratedResults();

                // Vyber smer s väčším priestorom
                turn_direction_ = (right > left) ? 1 : -1;

                RCLCPP_INFO(this->get_logger(), "Prekážka vpredu, zastavenie. Začínam otáčať o 90° do %s strany",
                            (turn_direction_ == 1 ? "lava" : "prava"));
                return;
            }

            // Jazda PID reguláciou (nezmenené)
            float error = left - right;
            float error_avg = (error + last_error) / 2.0f;
            float d_error = error_avg - last_error;
            integral_ += error_avg;

            float integral_saturation = 2.0f;
            integral_ = std::clamp(integral_, -integral_saturation, integral_saturation);

            float speed_diff = Kp * error_avg + Kd * d_error + Ki * integral_;
            speed_diff = std::clamp(speed_diff, 128.0f - base_speed, base_speed - 128.0f);

            if (left>side_threshold || right>side_threshold){
                left_motor_speed = base_speed;
                right_motor_speed = base_speed;
            }
            else if (error < 0.2)
            {
                left_motor_speed = base_speed - speed_diff;
                right_motor_speed = base_speed + speed_diff;
            }
            else {
                left_motor_speed = base_speed - speed_diff;
                right_motor_speed = base_speed + speed_diff;
            }

            motor_controller_->set_motor_speeds({
                std::clamp(left_motor_speed, 128.0f, 2 * base_speed - 128.0f),
                std::clamp(right_motor_speed, 128.0f, 2 * base_speed - 128.0f)
            });


            last_error = error;

            RCLCPP_INFO(this->get_logger(), "[DRIVE] L: %.2f R: %.2f F: %.2f Err: %.2f", left, right, front, error);
        }
        else if (state_ == DriveState::TURNING) {
            float current_yaw = imu_node_->getIntegratedResults();
            float delta_yaw = current_yaw - turn_start_yaw_;

            // Normalizácia
            while (delta_yaw > PI) delta_yaw -= 2 * PI;
            while (delta_yaw < -PI) delta_yaw += 2 * PI;

            float target_yaw = turn_direction_ * ((PI / 2.0f)- PI/18);  // ±90°

            if (std::abs(delta_yaw) >= std::abs(target_yaw) * 0.92f) {
                // Otočené

                motor_controller_->set_motor_speeds({128, 128});

                state_ = DriveState::DRIVE_FORWARD;
                last_error = 0;
                integral_ = 0;
                RCLCPP_INFO(this->get_logger(), "Otočka dokončená. Pokračujem dopredu.");
                return;
            }

            // Otáčaj podľa smeru
            float turn_speed = 3.0f;
            if (turn_direction_ == 1) {
                motor_controller_->set_motor_speeds({128 + turn_speed, 128 - turn_speed});
            } else {
                motor_controller_->set_motor_speeds({128 - turn_speed, 128 + turn_speed});
            }


            RCLCPP_INFO(this->get_logger(), "[TURNING] Yaw: %.2f ΔYaw: %.2f", current_yaw, delta_yaw);
        }
    }

}

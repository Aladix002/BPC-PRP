#include "nodes/pid.hpp"
#include <algorithm>
#include <cmath>

namespace nodes {

    constexpr float PI = 3.14159265f;

    PidNode::PidNode(std::shared_ptr<ImuNode> imu_node)
        : Node("pid_node"),
          Kp(11), Kd(9), Ki(0),
          base_speed(145.0f),
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

        left_line_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/left_line_sensor", 10,
            std::bind(&PidNode::left_line_callback, this, std::placeholders::_1)
        );

        right_line_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/right_line_sensor", 10,
            std::bind(&PidNode::right_line_callback, this, std::placeholders::_1)
        );

        motor_controller_ = std::make_shared<MotorController>();
    }

    bool just_turned = false;

    rclcpp::Time drive_forward_start_time_;
    float drive_forward_duration_threshold_ = 2.0f; // čas v sekundách po ktorom resetujeme just_turned
    rclcpp::Time left_line_time_;
    rclcpp::Time right_line_time_;

    void PidNode::left_line_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            left_line_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "[LINE] Left line detected.");
        }
    }

    void PidNode::right_line_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            right_line_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "[LINE] Right line detected.");
        }
    }

    void PidNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        LidarFiltr filter;
        auto result = filter.apply_filter(msg->ranges, msg->angle_min, msg->angle_max, msg->range_min, msg->range_max);

        float front = result.front;
        float front_left = result.front_left;
        float front_right = result.front_right;
        float back_left = result.back_left;
        float back_right = result.back_right;
        float right_side = result.right_side;

        float side_threshold = 0.18f;
        float front_side_threshold = 0.4f;
        float front_side_RT_threshold = 0.18f;
        float left_motor_speed = 0, right_motor_speed = 0;
        float front_threshold = 0.3f;
        bool full_turn = false;

        if (state_ == DriveState::DRIVE_FORWARD) {
            // Ak sme práve prešli do tohto stavu, zaznamenáme čas
            static bool first_time_in_drive = true;
            if (first_time_in_drive) {
                drive_forward_start_time_ = this->now();
                first_time_in_drive = false;
            }

            // Skontrolujeme, či uplynul dostatočný čas na reset just_turned
            auto current_time = this->now();
            auto time_in_drive_forward = (current_time - drive_forward_start_time_).seconds();

            if (time_in_drive_forward > drive_forward_duration_threshold_) {
                just_turned = false;
                RCLCPP_INFO(this->get_logger(), "Reset just_turned after %.2f seconds", time_in_drive_forward);
            }

            static bool waiting_before_turn = false;
            static rclcpp::Time wait_start_time;

            if ((front < front_threshold) || ((right_side > front_side_threshold) && !just_turned)) {
                // Ak sa ešte čaká pred otočením
                if ((right_side > front_side_threshold) && !just_turned && !waiting_before_turn) {
                    wait_start_time = this->now();
                    waiting_before_turn = true;

                    // Choď ďalej dopredu na chvíľu
                    motor_controller_->set_motor_speeds({base_speed, base_speed});
                    RCLCPP_INFO(this->get_logger(), "Zistená prekážka vpravo – čakám pred otočkou...");
                    return;
                }

                // Ak už čakáme – skontroluj či uplynulo 0.5 sekundy
                if (waiting_before_turn) {
                    auto time_waiting = (this->now() - wait_start_time).seconds();
                    if (time_waiting < 0.5) {
                        // Ešte pokračuj dopredu
                        motor_controller_->set_motor_speeds({base_speed, base_speed});
                        return;
                    } else {
                        waiting_before_turn = false;
                    }
                }

                // Zastavíme pred otočkou
                motor_controller_->set_motor_speeds({128.0f, 128.0f});

                // Pripravíme otočku
                state_ = DriveState::TURNING;
                turn_start_yaw_ = imu_node_->getIntegratedResults();
                first_time_in_drive = true; // Reset pre ďalšie DRIVE_FORWARD

                if (right_side > front_side_threshold) {
                    turn_direction_ = 1;
                    just_turned = true;
                } else if ((back_left < front_side_threshold) && (back_right < front_side_threshold)) {
                    turn_direction_ = 2;
                    full_turn = true;
                } else {
                    turn_direction_ = -1;
                }

                RCLCPP_INFO(this->get_logger(), "Prekážka vpredu – začínam otáčať o 90° do %s strany",
                            (turn_direction_ == 1 ? "lava" : "prava"));
                return;
            }

            // Jazda PID reguláciou
            float error = back_left - back_right;
            float error_avg = (error + last_error) / 2.0f;
            float d_error = error_avg - last_error;
            integral_ += error_avg;

            float integral_saturation = 2.0f;
            integral_ = std::clamp(integral_, -integral_saturation, integral_saturation);

            float speed_diff = Kp * error_avg + Kd * d_error + Ki * integral_;

            if (front_left < side_threshold) {
                speed_diff -= 1.5;
            }
            if (front_right < side_threshold) {
                speed_diff += 1.5;
            }

            RCLCPP_INFO(this->get_logger(), "[DRIVE] L: %.2f R: %.2f F: %.2f Err: %.2f RS: %.2f", front_left, front_right, front, error, right_side);

            speed_diff = std::clamp(speed_diff, 128.0f - base_speed, base_speed - 128.0f);

            if (back_left > front_side_threshold || back_right > front_side_threshold) {
                left_motor_speed = base_speed;
                right_motor_speed = base_speed;
            } else if ((front > front_side_threshold && front_left > front_side_threshold) || (front > front_side_threshold && back_left > front_side_threshold)) {
                left_motor_speed = base_speed;
                right_motor_speed = base_speed;
            } else {
                left_motor_speed = base_speed - speed_diff;
                right_motor_speed = base_speed + speed_diff;
            }

            motor_controller_->set_motor_speeds({
                std::clamp(left_motor_speed, 128.0f, 2 * (base_speed - 128.0f) + 128.0f),
                std::clamp(right_motor_speed, 128.0f, 2 * (base_speed - 128.0f) + 128.0f)
            });

            last_error = error;

            RCLCPP_INFO(this->get_logger(), "[DRIVE] L: %.2f R: %.2f F: %.2f Err: %.2f", front_left, front_right, front, error);
        } else if (state_ == DriveState::TURNING) {
            float current_yaw = imu_node_->getIntegratedResults();
            float delta_yaw = current_yaw - turn_start_yaw_;

            // Normalizácia
            while (delta_yaw > PI) delta_yaw -= 2 * PI;
            while (delta_yaw < -PI) delta_yaw += 2 * PI;

            float target_yaw = imu_node_->getIntegratedResults();
            if (full_turn) {
                target_yaw = turn_direction_ * (PI / 2.0f);
                full_turn = false;
            } else {
                target_yaw = turn_direction_ * ((PI / 2.0f) - PI / 18.0f);
            }

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
            float turn_speed = 10.0f;
            if (turn_direction_ == 1) {
                motor_controller_->set_motor_speeds({128 + turn_speed, 128 - turn_speed});
            } else {
                motor_controller_->set_motor_speeds({128 - turn_speed, 128 + turn_speed});
            }

            RCLCPP_INFO(this->get_logger(), "[TURNING] Yaw: %.2f ΔYaw: %.2f", current_yaw, delta_yaw);
        }
    }

}
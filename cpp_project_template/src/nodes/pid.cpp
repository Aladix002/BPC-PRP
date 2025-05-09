#include "nodes/pid.hpp"
#include <algorithm>
#include <cmath>
#include "std_msgs/msg/int32.hpp"

namespace nodes {

    constexpr float PI = 3.14159265f;

    PidNode::PidNode(std::shared_ptr<ImuNode> imu_node)
        : Node("pid_node"),
          Kp(7.0f), Kd(5.0f), Ki(0), // Ki se nepouziva, tak ani nenastavuj hoidnotu
          base_speed(142.0f),
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

        aruco_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/bpc_prp_robot/tag_detected", 10,
        std::bind(&PidNode::aruco_callback, this, std::placeholders::_1)
    );

        motor_controller_ = std::make_shared<MotorController>();
    }
    bool just_turned = false;
    bool just_turned_left = false;
    bool just_turned_right = false;


    rclcpp::Time drive_forward_start_time_;
    float drive_forward_duration_threshold_ = 4.0f; // čas v sekundách po ktorom resetujeme just_turned

void PidNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    LidarFiltr filter;
    auto result = filter.apply_filter(msg->ranges, msg->angle_min, msg->angle_max, msg->range_min, msg->range_max);

    float front = result.front;
    float front_left = result.front_left;
    float front_right = result.front_right;
    float back_left = result.back_left;
    float back_right = result.back_right;
    float right_side = result.right_side;
    float left_side = result.left_side;

    float left_side_follow_front = result.left_side_follow_front;
    float left_side_follow_back = result.left_side_follow_back;
    float right_side_follow_front = result.right_side_follow_front;
    float right_side_follow_back = result.right_side_follow_back;


    float side_threshold=0.18f;
    float front_side_threshold = 0.35f;
    float front_side_RT_threshold = 0.18f;
    float left_motor_speed=0, right_motor_speed=0;
    float front_treshold = 0.25f;
    bool full_turn = false;
    static bool waiting_before_turn = false;

    RCLCPP_INFO(this->get_logger(), "Last Scanned ArUco ID: %d", last_aruco_id_);

    if (state_ == DriveState::DRIVE_FORWARD)
    {
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
            just_turned_left = false;
            just_turned_right = false;
            RCLCPP_INFO(this->get_logger(), "Reset just_turned after %.2f seconds", time_in_drive_forward);
        }


        static rclcpp::Time wait_start_time;

        if (front < front_treshold) {
            // pred otockou zastavim
            motor_controller_->set_motor_speeds({128.0f, 128.0f});

            // Pripravíme otočku
            state_ = DriveState::TURNING;
            turn_start_yaw_ = imu_node_->getIntegratedResults();
            first_time_in_drive = true; // Reset pre ďalšie DRIVE_FORWARD
            waiting_before_turn = false;

            if (back_right > front_side_threshold) {
                turn_direction_ = 1;
                just_turned_left=false;
                just_turned = true;
                just_turned_right = true;
                RCLCPP_INFO(this->get_logger(), "Prekážka vpredu – začínam otáčať o 90° do prave strany");
            }
            else if ((back_left < front_side_threshold) && (back_right < front_side_threshold)) {
                turn_direction_ = 2;
                full_turn = true;
                RCLCPP_INFO(this->get_logger(), "OTACAM O 180");
            }
            else {
                turn_direction_ = -1;
                just_turned_right=false;
                just_turned = true;
                just_turned_left = true;
                RCLCPP_INFO(this->get_logger(), "Prekážka vpredu – začínam otáčať o 90° do leve strany");
            }
            return;
        }
        else{
            // Ak sa ešte čaká pred otočením
            if ((right_side > front_side_threshold) && !just_turned && !waiting_before_turn) {
                wait_start_time = this->now();
                waiting_before_turn = true;
                RCLCPP_INFO(this->get_logger(), "Zistená prekážka vpravo – čakám pred otočkou...");
            }
            // Ak už čakáme – skontroluj či uplynulo 0.5 sekundy
            if (waiting_before_turn) {
                auto time_waiting = (this->now() - wait_start_time).seconds();
                if (time_waiting < 0.5f) {
                    // ridim se jednou ze sten regulatorem steny
                    //return;
                }
                else {
                    waiting_before_turn = false;
                    motor_controller_->set_motor_speeds({128.0f, 128.0f});
                    state_ = DriveState::TURNING;
                    turn_start_yaw_ = imu_node_->getIntegratedResults();
                    first_time_in_drive = true; // Reset pre ďalšie DRIVE_FORWARD
                    //if (right_side > front_side_threshold) {
                    turn_direction_ = 1;
                    just_turned_left=false;
                    just_turned = true;
                    just_turned_right = true;
                    //}
                    RCLCPP_INFO(this->get_logger(), "Vpravo je cesta, tak tam zatocim");
                    return;
                }
            }

            float error;// = back_left - back_right;
            float error_avg;// = (error + last_error) / 2.0f;
            float d_error;// = error_avg - last_error;
            float speed_diff;// = Kp * error_avg + Kd * d_error + Ki * integral_;
            //if (((front > front_side_threshold && front_left > front_side_threshold) && ((front > front_side_threshold && front_right > front_side_threshold))
            //   || (back_left>front_side_threshold && back_right>front_side_threshold))){

            //ridim v koridoru
            if (just_turned_left)
            {
                RCLCPP_INFO(this->get_logger(), "Drzim se prave steny po otocce");
                //error = 0.20f - right_side;
                error=(0.8f*right_side_follow_front-right_side_follow_back);//+0.20f
                if (error>2.0f){error=2.0f;}
                if (error<-2.0f){error=-2.0f;}
                if (std::isnan(error)){error=last_error;}

                error_avg = (error + last_error) / 2.0f;
                d_error = error_avg - last_error;
                speed_diff = Kp * error_avg + Kd * d_error;
            }
            else if (just_turned_right)
            {
                RCLCPP_INFO(this->get_logger(), "Drzim se leve steny po otocce");
                error=(left_side_follow_back-0.8f*left_side_follow_front);//+0.20f;
                if (error>2.0f){error=2.0f;}
                if (error<-2.0f){error=-2.0f;}
                if (std::isnan(error)){error=last_error;}
                //error = left_side - 0.20f;
                error_avg = (error + last_error) / 2.0f;
                d_error = error_avg - last_error;
                speed_diff = Kp * error_avg + Kd * d_error;
            }
            else if (((front > front_side_threshold && left_side > front_side_threshold) && ((front > front_side_threshold && right_side > front_side_threshold))
                || (back_left>front_side_threshold && back_right>front_side_threshold)))
            {
                //front_side_threshold

                speed_diff=0;
                RCLCPP_INFO(this->get_logger(), "Vidim krizovatku +, jedu rovne");
            }
            else if ((front_left <= front_side_threshold && front_right <= front_side_threshold))
            {
                error = back_left - back_right;
                error_avg = (error + last_error) / 2.0f;
                d_error = error_avg - last_error;
                speed_diff = Kp * error_avg + Kd * d_error + Ki * integral_;
                RCLCPP_INFO(this->get_logger(), "Ridim se obema regulatory");
            }
            // risim se jenom levou stenou
            else if (left_side <= front_side_threshold && left_side<right_side)//front > front_side_threshold &&
            {
                RCLCPP_INFO(this->get_logger(), "Drzim se leve steny");
                error=(left_side_follow_back-0.8f*left_side_follow_front);
                if (error>2.0f){error=2.0f;}
                if (error<-2.0f){error=-2.0f;}
                if (std::isnan(error)){error=last_error;}
                //error = left_side - 0.20f;
                error_avg = (error + last_error) / 2.0f;
                d_error = error_avg - last_error;
                speed_diff = Kp * error_avg + Kd * d_error;
            }
            // ridim se jenom pravou stenou
            else if (right_side <= front_side_threshold)
            {
                RCLCPP_INFO(this->get_logger(), "Drzim se prave steny");
                //error = 0.20f - right_side;
                error=(0.8f*right_side_follow_front-right_side_follow_back);
                if (error>2.0f){error=2.0f;}
                if (error<-2.0f){error=-2.0f;}
                if (std::isnan(error)){error=last_error;}

                error_avg = (error + last_error) / 2.0f;
                d_error = error_avg - last_error;
                speed_diff = Kp * error_avg + Kd * d_error;
            }



            /*
            // else ridime podle vypocitaneho
            else {
                RCLCPP_INFO(this->get_logger(), "Ridim regulatorem v koridoru");
                error = back_left - back_right;
                error_avg = (error + last_error) / 2.0f;
                d_error = error_avg - last_error;
                speed_diff = Kp * error_avg + Kd * d_error;

            }*/

            if (speed_diff!=0)
            {
                if (front_left<side_threshold){speed_diff-=3.0f;}
                if (front_right<side_threshold){speed_diff+=3.0f;}
            }

            //speed_diff = std::clamp(speed_diff, 128.0f - base_speed, base_speed - 128.0f);
            //speed_diff = std::clamp(speed_diff, 128.0f+5.0f - base_speed, base_speed - (128.0f+5.0f));
            speed_diff = std::clamp(speed_diff,-6.0f,6.0f);


            left_motor_speed = base_speed - speed_diff;
            right_motor_speed = base_speed + speed_diff;

            motor_controller_->set_motor_speeds({
                std::clamp(left_motor_speed, 128.0f, 2 * (base_speed - 128.0f)+128.0f),
                std::clamp(right_motor_speed, 128.0f, 2 *(base_speed - 128.0f)+128.0f)
            });


            last_error = error;
            //RCLCPP_INFO(this->get_logger(), "[DRIVE] L: %.2f R: %.2f F: %.2f Err: %.2f", front_left, front_right, front, error);
            RCLCPP_INFO(this->get_logger(), "[DRIVE] RF: %.2f RB: %.2f F: %.2f Err: %.2f", right_side_follow_front, right_side_follow_back, front, error);

        }
    }
        else if (state_ == DriveState::TURNING) {
            float current_yaw = imu_node_->getIntegratedResults();
            float delta_yaw = current_yaw - turn_start_yaw_;

            // Normalizácia
            while (delta_yaw > PI) delta_yaw -= 2 * PI;
            while (delta_yaw < -PI) delta_yaw += 2 * PI;

            float target_yaw = imu_node_->getIntegratedResults();
            if (full_turn) {
                 target_yaw = turn_direction_ * (PI / 2.0f);
                full_turn = false;
            }
            else {
                 target_yaw = turn_direction_ * ((PI / 2.0f) - PI/18.0f);
            }


            if (std::abs(delta_yaw) >= std::abs(target_yaw) * 0.97f) {
                // Otočené

                motor_controller_->set_motor_speeds({128, 128});

                state_ = DriveState::DRIVE_FORWARD;
                last_error = 0;
                integral_ = 0;
                RCLCPP_INFO(this->get_logger(), "Otočka dokončená. Pokračujem dopredu.");
                return;
            }

            // Otáčaj podľa smeru
            float turn_speed = 5.0f;
            if (turn_direction_ == 1) {
                motor_controller_->set_motor_speeds({128 + turn_speed, 128 - turn_speed});
            } else {
                motor_controller_->set_motor_speeds({128 - turn_speed, 128 + turn_speed});
            }


            //RCLCPP_INFO(this->get_logger(), "[TURNING] Yaw: %.2f ΔYaw: %.2f", current_yaw, delta_yaw);
        }
    }

    void PidNode::aruco_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Aruco callback invoked!");
    last_aruco_id_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received ArUco ID: %d", last_aruco_id_);
}

}

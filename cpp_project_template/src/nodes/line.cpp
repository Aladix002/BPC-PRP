#include "nodes/line.hpp"

using namespace nodes;

LineSensorListener::LineSensorListener()
    : Node("line_sensor_listener"),
      Kp(0.07), Kd(0.0015), Ki(0.001), base_speed(132), last_error(0.0), integral_(0.0f)
{
    line_sensors_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
        "/bpc_prp_robot/line_sensors", 10,
        std::bind(&LineSensorListener::on_line_sensors_msg, this, std::placeholders::_1));

    left_line_pub_ = this->create_publisher<std_msgs::msg::Bool>("/left_line_sensor", 10);
    right_line_pub_ = this->create_publisher<std_msgs::msg::Bool>("/right_line_sensor", 10);

    motor_controller_ = std::make_shared<MotorController>();
}

void LineSensorListener::reset_integral() {
    integral_ = 0.0f;
}

void LineSensorListener::on_line_sensors_msg(std::shared_ptr<std_msgs::msg::UInt16MultiArray> msg)
{
    if (msg->data.size() >= 2)
    {
        float left_value = static_cast<float>(msg->data[0]);
        float right_value = static_cast<float>(msg->data[1]);

        // Publikuj detekciu čiary
        std_msgs::msg::Bool left_msg;
        std_msgs::msg::Bool right_msg;
        left_msg.data = (left_value > 600);
        right_msg.data = (right_value > 600);
        left_line_pub_->publish(left_msg);
        right_line_pub_->publish(right_msg);

        float error = left_value - right_value;
        float error_avg = error + last_error / 2;
        float d_error = error_avg - last_error;

        // Integrálna limitácia
        integral_ += error_avg;
        if (integral_ > 2000) integral_ = 2000;
        if (integral_ < -2000) integral_ = -2000;

        RCLCPP_INFO(this->get_logger(), "Left: %f, Right: %f, Int: %f", left_value, right_value, integral_);

        float speed_diff = (Kp * error_avg + Kd * d_error + Ki * integral_);
        if (speed_diff > base_speed - 128) speed_diff = base_speed - 128;
        else if (speed_diff < 128 - base_speed) speed_diff = 128 - base_speed;

        float right_motor_speed;
        float left_motor_speed;
        float sensor_threshhold = 300;

        if (error == 0 || (left_value > sensor_threshhold && right_value > sensor_threshhold)) {
            right_motor_speed = base_speed;
            left_motor_speed = base_speed;
        }
        else if (error > 0) {
            right_motor_speed = base_speed + 0.3f * speed_diff;
            left_motor_speed = base_speed - speed_diff;
        }
        else {
            right_motor_speed = base_speed + speed_diff;
            left_motor_speed = base_speed - 0.3f * speed_diff;
        }

        motor_controller_->set_motor_speeds({
            std::clamp(left_motor_speed, 0.0f, 255.0f),
            std::clamp(right_motor_speed, 0.0f, 255.0f)
        });

        last_error = error;
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Invalid sensor data received.");
    }
}



#include "rclcpp/rclcpp.hpp"
#include "nodes/button.hpp"
#include "nodes/lidar.hpp"
#include "nodes/imu_node.hpp"
#include "nodes/pid.hpp"
#include "nodes/camera.hpp"

using namespace nodes;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto button_listener = std::make_shared<ButtonListener>();
    auto imu_node = std::make_shared<ImuNode>();
    auto pid_node = std::make_shared<PidNode>(imu_node);
    auto camera_node = std::make_shared<CameraNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(button_listener);
    executor.add_node(imu_node);
    executor.add_node(camera_node);


    imu_node->setMode(ImuNodeMode::CALIBRATE);
    RCLCPP_INFO(imu_node->get_logger(), "Kalibrace IMU zahájena...");

    auto start_time = imu_node->now();
    bool was_active = false;
    auto executor_thread = std::thread([&executor](){executor.spin();});

    while (rclcpp::ok())
    {
        auto now = imu_node->now();
        auto elapsed = (now - start_time).seconds();

        if (elapsed >= 2.0 && imu_node->getMode() == ImuNodeMode::CALIBRATE)
        {
            imu_node->setMode(ImuNodeMode::INTEGRATE);
            RCLCPP_INFO(imu_node->get_logger(), "Kalibrace dokončena. Přepínám do INTEGRATE módu.");
        }

        if (imu_node->getMode() == ImuNodeMode::INTEGRATE && static_cast<int>(elapsed) % 1 == 0)
        {
            auto yaw = imu_node->getIntegratedResults();
        }

        bool is_active = button_listener->is_active();

        if (is_active && !was_active)
        {
            RCLCPP_INFO(button_listener->get_logger(), "Spúšťam PidNode");
            executor.add_node(pid_node);
        }
        else if (!is_active && was_active)
        {
            RCLCPP_INFO(button_listener->get_logger(), "Zastavujem PidNode");
            executor.remove_node(pid_node);
        }

        was_active = is_active;
    }

    rclcpp::shutdown();
    return 0;
}




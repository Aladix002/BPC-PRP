#ifndef CAMERA_NODE_HPP
#define CAMERA_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "aruco_detector.hpp"

using namespace nodes;

class CameraNode : public rclcpp::Node {
public:
    CameraNode();

private:
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr aruco_id_publisher_;
    image_transport::Publisher image_publisher_;
    image_transport::ImageTransport it_;
    int last_aruco_id_;
};

#endif // CAMERA_NODE_HPP





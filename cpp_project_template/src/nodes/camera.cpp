#include "nodes/camera.hpp"

CameraNode::CameraNode() : Node("camera_node"), it_(std::make_shared<rclcpp::Node>("camera_node")) {
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/bpc_prp_robot/camera/compressed", 10,
        std::bind(&CameraNode::image_callback, this, std::placeholders::_1));

    image_publisher_ = it_.advertise("/bpc_prp_robot/camera/processed", 1);
}

void CameraNode::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    try {
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Received an empty image.");
            return;
        }

        ArucoDetector aruco_detector;
        std::vector<ArucoDetector::Aruco> detected_markers = aruco_detector.detect(frame);

        if (!detected_markers.empty()) {
            std::vector<std::vector<cv::Point2f>> corners;
            for (const auto& marker : detected_markers) {
                corners.push_back(marker.corners);
            }
            cv::aruco::drawDetectedMarkers(frame, corners);
        }

        sensor_msgs::msg::Image::SharedPtr processed_image_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

        image_publisher_.publish(processed_image_msg);

    } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error converting image: %s", e.what());
    }
}







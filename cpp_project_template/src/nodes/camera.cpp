#include "nodes/camera.hpp"
#include "std_msgs/msg/int32.hpp"

CameraNode::CameraNode()
    : Node("camera_node"), it_(std::make_shared<rclcpp::Node>("camera_node")) {

    // Image subscriber to receive the camera images
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/bpc_prp_robot/camera/compressed", 10,
        std::bind(&CameraNode::image_callback, this, std::placeholders::_1));

    // Image publisher for processed camera images
    image_publisher_ = it_.advertise("/bpc_prp_robot/camera/processed", 1);

    // ArUco ID publisher to send the last detected ArUco ID
    aruco_id_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/bpc_prp_robot/tag_detected", 10);
}

void CameraNode::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    try {
        // Decode the received image from the compressed message
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Received an empty image.");
            return;
        }

        // Initialize the ArUco detector
        ArucoDetector aruco_detector;
        std::vector<ArucoDetector::Aruco> detected_markers = aruco_detector.detect(frame);

        if (!detected_markers.empty()) {
            // Store the last detected ArUco ID
            last_aruco_id_ = detected_markers.back().id;

            // Output the detected ArUco tag IDs
            for (const auto& marker : detected_markers) {
                std::cout << "Detected ArUco marker ID: " << marker.id << std::endl;
            }

            // Publish the last detected ArUco ID
            std_msgs::msg::Int32 aruco_msg;
            aruco_msg.data = last_aruco_id_;
            aruco_id_publisher_->publish(aruco_msg);
        }

        // Convert the processed image to a ROS image message
        sensor_msgs::msg::Image::SharedPtr processed_image_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

        // Publish the processed image
        image_publisher_.publish(processed_image_msg);

    } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error converting image: %s", e.what());
    }
}









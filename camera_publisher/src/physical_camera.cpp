#include "physical_camera.h"

CameraPublisher::CameraPublisher(ros::NodeHandle& nh) : nh_(nh), it_(nh_) {
    // Get the topic name from the parameter server
    nh_.param("camera_topic", topic_name_, std::string("/camera/image"));

    // Initialize the publisher
    image_pub_ = it_.advertise(topic_name_, 1);

    // Open the camera
    cap_.open(0, cv::CAP_V4L2);
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    if (!cap_.isOpened()) {
        ROS_ERROR("Failed to open camera");
        ros::shutdown();
    }
    else {
        ROS_INFO("Camera opened successfully");
    }
}

void CameraPublisher::publishImage() {
    cv::Mat frame;
    cap_ >> frame; // Capture a frame

    if (!frame.empty()) {
        // Convert the frame to a ROS image message
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        image_pub_.publish(msg);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_publisher");
    ros::NodeHandle nh;

    CameraPublisher camera_publisher(nh);

    // ros::Rate loop_rate(30); // 10 Hz
    while (ros::ok()) {
        camera_publisher.publishImage();
        ros::spinOnce();
        cv::waitKey(5);
        // loop_rate.sleep();
    }

    return 0;
}
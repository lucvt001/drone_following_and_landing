#ifndef ARUCO_DETECTOR_H
#define ARUCO_DETECTOR_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <unordered_map>

class ArucoDetector {
public:
    ArucoDetector(ros::NodeHandle& nh);

private:
    struct MarkerInfo {
        float x;
        float y;
        float size;
    };
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void loadCameraParameters(const std::string& filename);
    void loadMarkerInfo(const std::string& filename);
    void displayAndPublishImage(cv::Mat& frame);

    std::string camera_parameters_yaml_;
    std::string marker_info_yaml_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    std::unordered_map<int, MarkerInfo> marker_info_;

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher bbox_pub_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Mat frame;

    std::string input_image_topic_;
    std::string output_image_topic_;
    std::string bbox_pub_topic_;
    bool is_display_;

    const int frame_width = 640;
    const int frame_height = 480;
};

#endif // ARUCO_DETECTOR_H
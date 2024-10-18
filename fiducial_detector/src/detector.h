#ifndef ARUCO_DETECTOR_H
#define ARUCO_DETECTOR_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class ArucoDetector {
public:
    ArucoDetector(ros::NodeHandle& nh);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher bbox_pub_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;

    const int frame_width = 640;
    const int frame_height = 480;
};

#endif // ARUCO_DETECTOR_H
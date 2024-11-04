#ifndef ARUCO_DETECTOR_H
#define ARUCO_DETECTOR_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "camera.h"

class ArucoDetector {
public:
    ArucoDetector(ros::NodeHandle& nh, int camera_index = 0);
    void run();

private:
    ros::NodeHandle nh_;
    ros::Publisher bbox_pub_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    Camera camera_;
};

#endif // ARUCO_DETECTOR_H
#include <fiducial_detector/detector.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_detector");
    ros::NodeHandle nh("~");

    ArucoDetector aruco_detector(nh);

    ros::spin();
    return 0;
}
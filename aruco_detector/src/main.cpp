#include <aruco_detector/detector.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_detector");
    ros::NodeHandle nh("~");
    
    // Get the camera index from the parameter server
    int camera_index;
    nh.param("camera_index", camera_index, 0); // Default to 0 if not set

    ArucoDetector aruco_detector(nh, camera_index);

    aruco_detector.run();
    return 0;
}
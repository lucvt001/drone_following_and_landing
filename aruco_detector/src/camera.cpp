#include <aruco_detector/camera.h>
#include <iostream>

Camera::Camera(int camera_index) {
    // Open the camera
    cap_.open(0, cv::CAP_V4L2);
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    if (!cap_.isOpened()) {
        std::cerr << "Error: Could not open camera with index " << camera_index << std::endl;
    }
    frame_height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
    frame_width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
}

Camera::~Camera() {
    // Release the camera
    if (cap_.isOpened()) {
        cap_.release();
    }
}

cv::Mat Camera::get_frame() {
    cv::Mat frame;
    if (cap_.isOpened()) {
        cap_ >> frame; // Capture a frame
    } else {
        std::cerr << "Error: Camera is not opened" << std::endl;
    }
    return frame;
}
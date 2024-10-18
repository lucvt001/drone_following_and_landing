#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>

class Camera {
public:
    Camera(int camera_index = 0);
    ~Camera();
    cv::Mat get_frame();

    int frame_width = 640;
    int frame_height = 480;

private:
    cv::VideoCapture cap_;
};

#endif // CAMERA_H
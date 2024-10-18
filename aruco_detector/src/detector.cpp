#include <aruco_detector/detector.h>
#include <aruco_detector/camera.h>

ArucoDetector::ArucoDetector(ros::NodeHandle& nh, int camera_index) : nh_(nh), camera_(camera_index) {
    // Initialize the dictionary for ArUco markers
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);

    // Initialize the publisher for bounding boxes
    bbox_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/aruco/bounding_boxes", 1);
}

void ArucoDetector::run() {
    int frame_width = camera_.frame_width;
    int frame_height = camera_.frame_height;

    while (ros::ok()) {
        // Convert the ROS image message to an OpenCV image
        cv::Mat frame = camera_.get_frame();

        // Detect ArUco markers
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        cv::aruco::detectMarkers(frame, dictionary_, markerCorners, markerIds);

        // Draw detected markers on the image
        if (!markerIds.empty()) {
            cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

            // Prepare the Float32MultiArray message
            std_msgs::Float32MultiArray bbox_msg;
            bbox_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
            bbox_msg.layout.dim[0].size = markerIds.size();
            bbox_msg.layout.dim[0].stride = 4;
            bbox_msg.layout.dim[0].label = "bounding_boxes";

            // Calculate bounding boxes and populate the message
            for (const auto& corners : markerCorners) {
                float x_min = std::min({corners[0].x, corners[1].x, corners[2].x, corners[3].x});
                float y_min = std::min({corners[0].y, corners[1].y, corners[2].y, corners[3].y});
                float x_max = std::max({corners[0].x, corners[1].x, corners[2].x, corners[3].x});
                float y_max = std::max({corners[0].y, corners[1].y, corners[2].y, corners[3].y});
                float width = x_max - x_min;
                float height = y_max - y_min;

                // Normalize the coordinates
                float x_min_norm = x_min / frame_width;
                float y_min_norm = y_min / frame_height;
                float width_norm = width / frame_width;
                float height_norm = height / frame_height;

                bbox_msg.data.push_back(x_min_norm);
                bbox_msg.data.push_back(y_min_norm);
                bbox_msg.data.push_back(width_norm);
                bbox_msg.data.push_back(height_norm);
            }

            // Publish the bounding boxes
            bbox_pub_.publish(bbox_msg);
        }

        // Display the image with detected markers
        cv::imshow("Aruco Detector", frame);
        
        // Check if 'q' key is pressed
        if (cv::waitKey(5) == 'q')
            break;

    } 
}
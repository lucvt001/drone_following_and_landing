#include "detector.h"

ArucoDetector::ArucoDetector(ros::NodeHandle& nh) : nh_(nh), it_(nh_) {
    nh_.getParam("input_image_topic", input_image_topic_);
    nh_.getParam("output_image_topic", output_image_topic_);
    nh_.getParam("bbox_pub_topic", bbox_pub_topic_);
    nh_.getParam("is_display", is_display_);

    // Subscribe to the image topic
    image_sub_ = it_.subscribe(input_image_topic_, 2, &ArucoDetector::imageCallback, this);
    image_pub_ = it_.advertise(output_image_topic_, 2);

    // Initialize the dictionary for ArUco markers
    // Initialize all predefined dictionaries
    dictionaries_ = {
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11)
    };

    // Initialize the publisher for bounding boxes
    bbox_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(bbox_pub_topic_, 1);
}

void ArucoDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // Convert the ROS image message to an OpenCV image
        frame = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Detect ArUco markers
        std::vector<int> all_marker_ids;
        std::vector<std::vector<cv::Point2f>> all_marker_corners;
        // Detect ArUco markers from all dictionaries
        for (const auto& dictionary : dictionaries_) {
            std::vector<int> marker_ids;
            std::vector<std::vector<cv::Point2f>> marker_corners;
            cv::aruco::detectMarkers(frame, dictionary, marker_corners, marker_ids);

            all_marker_ids.insert(all_marker_ids.end(), marker_ids.begin(), marker_ids.end());
            all_marker_corners.insert(all_marker_corners.end(), marker_corners.begin(), marker_corners.end());
        }

        // Draw detected markers on the image
        if (!all_marker_ids.empty()) {
            cv::aruco::drawDetectedMarkers(frame, all_marker_corners, all_marker_ids);

            // Prepare the Float32MultiArray message
            std_msgs::Float32MultiArray bbox_msg;
            bbox_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
            bbox_msg.layout.dim[0].size = all_marker_ids.size();
            bbox_msg.layout.dim[0].stride = 4;
            bbox_msg.layout.dim[0].label = "bounding_boxes";

            // Calculate bounding boxes and populate the message
            for (const auto& corners : all_marker_corners) {
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
        if (is_display_) {
            cv::imshow("Aruco Detector", frame);
            cv::waitKey(1);
        }
        sensor_msgs::ImagePtr labeled_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        image_pub_.publish(labeled_image_msg);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}
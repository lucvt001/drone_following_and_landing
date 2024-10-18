#ifndef OFFSET_RELATIVE_NED_POSITION_H
#define OFFSET_RELATIVE_NED_POSITION_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class OffsetRelativeNEDPosition
{
public:
    OffsetRelativeNEDPosition(ros::NodeHandle& nh);

private:
    ros::NodeHandle nh_;
    ros::Subscriber relative_pos_sub_;
    ros::Subscriber car_pose_sub_;
    ros::Publisher offset_relative_pos_pub_;

    std::string car_pose_topic_;
    std::string relative_position_topic_;
    std::string offset_relative_position_topic_;

    geometry_msgs::Point offset_;
    geometry_msgs::Quaternion vehicle_orientation_;

    void relativePositionCallback(const geometry_msgs::Point::ConstPtr& msg);
    void vehiclePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    geometry_msgs::Point transformOffsetToGlobalFrame(const geometry_msgs::Point& offset, const geometry_msgs::Quaternion& orientation);
};

#endif // OFFSET_RELATIVE_NED_POSITION_H
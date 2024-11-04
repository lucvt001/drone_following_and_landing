#ifndef RELATIVE_NED_POSITION_H
#define RELATIVE_NED_POSITION_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class RelativeNEDPosition
{
public:
    RelativeNEDPosition(ros::NodeHandle& nh);

private:
    ros::NodeHandle nh_;
    ros::Subscriber drone_pose_sub_;
    ros::Subscriber car_pose_sub_;
    ros::Publisher relative_position_pub_;

    std::string drone_pose_topic_;
    std::string car_pose_topic_;
    std::string relative_position_topic_;

    geometry_msgs::PoseStamped drone_pose_;
    geometry_msgs::PoseStamped car_pose_;

    void droneCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void carCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void publishRelativePosition();
};

#endif // RELATIVE_NED_POSITION_H
#ifndef FOLLOW_RELATIVE_NED_H
#define FOLLOW_RELATIVE_NED_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <pid_controller/pid.h>

class FollowRelativeNED
{
public:
    FollowRelativeNED(ros::NodeHandle& nh);

private:
    ros::NodeHandle nh_;
    ros::Subscriber offset_relative_position_sub_;
    ros::Publisher cmd_vel_pub_;
    geometry_msgs::Point offset_relative_position_;

    std::string cmd_vel_topic_;
    std::string offset_relative_position_topic_;

    void offsetRelativePositionCallback(const geometry_msgs::Point::ConstPtr& msg);

    PID forward_pid_ = PID(0,0,0,0,0,0);
    PID leftright_pid_ = PID(0,0,0,0,0,0);
    PID updown_pid_ = PID(0,0,0,0,0,0);
    PID yaw_pid_ = PID(0,0,0,0,0,0);
};

#endif // FOLLOW_RELATIVE_NED_H
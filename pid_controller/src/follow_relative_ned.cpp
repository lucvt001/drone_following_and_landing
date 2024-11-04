#include <pid_controller/follow_relative_ned.h>

FollowRelativeNED::FollowRelativeNED(ros::NodeHandle& nh) : nh_(nh)
{
    nh_.getParam("/drone/cmd_vel_world", cmd_vel_topic_);
    nh_.getParam("/car_to_drone/offset_relative_position_ned", offset_relative_position_topic_);

    // Initialize subscriber
    offset_relative_position_sub_ = nh_.subscribe(offset_relative_position_topic_, 2, &FollowRelativeNED::offsetRelativePositionCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 2);

    // Retrieve PID parameters for forward
    double kp_forward, ki_forward, kd_forward, dt_forward, max_forward, min_forward;
    nh_.getParam("forward/kp", kp_forward);
    nh_.getParam("forward/ki", ki_forward);
    nh_.getParam("forward/kd", kd_forward);
    nh_.getParam("forward/dt", dt_forward);
    nh_.getParam("forward/max", max_forward);
    nh_.getParam("forward/min", min_forward);

    // Retrieve PID parameters for leftright
    double kp_leftright, ki_leftright, kd_leftright, dt_leftright, max_leftright, min_leftright;
    nh_.getParam("leftright/kp", kp_leftright);
    nh_.getParam("leftright/ki", ki_leftright);
    nh_.getParam("leftright/kd", kd_leftright);
    nh_.getParam("leftright/dt", dt_leftright);
    nh_.getParam("leftright/max", max_leftright);
    nh_.getParam("leftright/min", min_leftright);

    // Retrieve PID parameters for updown
    double kp_updown, ki_updown, kd_updown, dt_updown, max_updown, min_updown;
    nh_.getParam("updown/kp", kp_updown);
    nh_.getParam("updown/ki", ki_updown);
    nh_.getParam("updown/kd", kd_updown);
    nh_.getParam("updown/dt", dt_updown);
    nh_.getParam("updown/max", max_updown);
    nh_.getParam("updown/min", min_updown);

    // Retrieve PID parameters for yaw
    double kp_yaw, ki_yaw, kd_yaw, dt_yaw, max_yaw, min_yaw;
    nh_.getParam("yaw/kp", kp_yaw);
    nh_.getParam("yaw/ki", ki_yaw);
    nh_.getParam("yaw/kd", kd_yaw);
    nh_.getParam("yaw/dt", dt_yaw);
    nh_.getParam("yaw/max", max_yaw);
    nh_.getParam("yaw/min", min_yaw);

    // Initialize PID controllers
    forward_pid_ = PID(kp_forward, ki_forward, kd_forward, dt_forward, max_forward, min_forward);
    leftright_pid_ = PID(kp_leftright, ki_leftright, kd_leftright, dt_leftright, max_leftright, min_leftright);
    updown_pid_ = PID(kp_updown, ki_updown, kd_updown, dt_updown, max_updown, min_updown);
    yaw_pid_ = PID(kp_yaw, ki_yaw, kd_yaw, dt_yaw, max_yaw, min_yaw);
}

void FollowRelativeNED::offsetRelativePositionCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    offset_relative_position_ = *msg;

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = forward_pid_.calculate(0, -offset_relative_position_.x);
    cmd_vel.linear.y = leftright_pid_.calculate(0, -offset_relative_position_.y);
    cmd_vel.linear.z = updown_pid_.calculate(0, -offset_relative_position_.z);
    cmd_vel_pub_.publish(cmd_vel);

    // ROS_INFO("Offset Relative Position: x: %f, linear_x: %f", offset_relative_position_.x, cmd_vel.linear.x);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "offset_relative_position_subscriber_node");
    ros::NodeHandle nh_("~");
    FollowRelativeNED offset_relative_position_subscriber(nh_);
    ros::spin();
    return 0;
}
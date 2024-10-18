#include <target_tracking_planner/relative_ned_position.h>

RelativeNEDPosition::RelativeNEDPosition(ros::NodeHandle& nh) : nh_(nh)
{
    nh_.getParam("/drone/pose", drone_pose_topic_);
    nh_.getParam("/car/pose", car_pose_topic_);
    nh_.getParam("/car_to_drone/relative_position_ned", relative_position_topic_);

    // Initialize subscribers
    drone_pose_sub_ = nh_.subscribe(drone_pose_topic_, 2, &RelativeNEDPosition::droneCallback, this);
    car_pose_sub_ = nh_.subscribe(car_pose_topic_, 2, &RelativeNEDPosition::carCallback, this);
    ROS_INFO("Subscribed to %s and %s", drone_pose_topic_.c_str(), car_pose_topic_.c_str());

    // Initialize publisher
    relative_position_pub_ = nh_.advertise<geometry_msgs::Point>(relative_position_topic_, 2);
}

void RelativeNEDPosition::droneCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    drone_pose_ = *msg;
}

void RelativeNEDPosition::carCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    car_pose_ = *msg;
    publishRelativePosition();
}

void RelativeNEDPosition::publishRelativePosition()
{
    geometry_msgs::Point relative_position;
    relative_position.x = car_pose_.pose.position.x - drone_pose_.pose.position.x;
    relative_position.y = car_pose_.pose.position.y - drone_pose_.pose.position.y;
    relative_position.z = car_pose_.pose.position.z - drone_pose_.pose.position.z;
    relative_position_pub_.publish(relative_position);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "relative_ned_position_node");
    ros::NodeHandle nh("~");
    RelativeNEDPosition relative_ned_position(nh);
    ros::spin();
    return 0;
}
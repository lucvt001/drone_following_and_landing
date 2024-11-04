#include <target_tracking_planner/offset_relative_ned_position.h>

OffsetRelativeNEDPosition::OffsetRelativeNEDPosition(ros::NodeHandle& nh) : nh_(nh)
{
    nh_.getParam("/car/pose", car_pose_topic_);
    nh_.getParam("/car_to_drone/relative_position_ned", relative_position_topic_);
    nh_.getParam("/car_to_drone/offset_relative_position_ned", offset_relative_position_topic_);

    // Initialize subscriber
    relative_pos_sub_ = nh_.subscribe(relative_position_topic_, 2, &OffsetRelativeNEDPosition::relativePositionCallback, this);
    car_pose_sub_ = nh_.subscribe(car_pose_topic_, 2, &OffsetRelativeNEDPosition::vehiclePoseCallback, this);

    // Initialize publisher
    offset_relative_pos_pub_ = nh_.advertise<geometry_msgs::Point>(offset_relative_position_topic_, 2);
    ROS_INFO("Publishing offset relative position to %s" , offset_relative_position_topic_.c_str());

    // Get the offset parameter
    nh_.param("offset_x", offset_.x, 0.0);
    nh_.param("offset_y", offset_.y, 0.0);
    nh_.param("offset_z", offset_.z, 0.0);
}

void OffsetRelativeNEDPosition::relativePositionCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    geometry_msgs::Point relative_position = *msg;

    // Transform the offset from the vehicle body frame to the global NED frame
    geometry_msgs::Point global_offset = transformOffsetToGlobalFrame(offset_, vehicle_orientation_);

    // Add the transformed offset to the relative position
    geometry_msgs::Point adjusted_relative_position;
    adjusted_relative_position.x = relative_position.x + global_offset.x;
    adjusted_relative_position.y = relative_position.y + global_offset.y;
    adjusted_relative_position.z = relative_position.z + global_offset.z;

    offset_relative_pos_pub_.publish(adjusted_relative_position);
}

void OffsetRelativeNEDPosition::vehiclePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    vehicle_orientation_ = msg->pose.orientation;
}

geometry_msgs::Point OffsetRelativeNEDPosition::transformOffsetToGlobalFrame(const geometry_msgs::Point& offset, const geometry_msgs::Quaternion& orientation)
{
    tf2::Quaternion quat;
    tf2::fromMsg(orientation, quat);

    tf2::Matrix3x3 rotation_matrix(quat);

    tf2::Vector3 offset_vector(offset.x, offset.y, offset.z);
    tf2::Vector3 transformed_offset = rotation_matrix * offset_vector;

    geometry_msgs::Point global_offset;
    global_offset.x = transformed_offset.x();
    global_offset.y = transformed_offset.y();
    global_offset.z = transformed_offset.z();

    return global_offset;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "offset_relative_ned_position_node");
    ros::NodeHandle nh("~");
    OffsetRelativeNEDPosition offset_relative_ned_position(nh);
    ros::spin();
    return 0;
}
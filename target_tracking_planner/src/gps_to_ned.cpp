#include <target_tracking_planner/gps_to_ned.h>

GPSToNED::GPSToNED(ros::NodeHandle& nh) : nh_(nh)
{
    nh_.getParam("/car/gps", car_gps_topic_);
    nh_.getParam("/drone/gps", drone_gps_topic_);
    nh_.getParam("/car_to_drone/relative_position_ned", car_to_drone_ned_topic_);

    // Initialize subscribers
    car_gps_sub_ = nh_.subscribe(car_gps_topic_, 10, &GPSToNED::carGPSCallback, this);
    drone_gps_sub_ = nh_.subscribe(drone_gps_topic_, 10, &GPSToNED::droneGPSCallback, this);

    // Initialize publisher
    ned_pub_ = nh_.advertise<geometry_msgs::Point>(car_to_drone_ned_topic_, 10);
}

void GPSToNED::carGPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    car_gps_.latitude = msg->latitude;
    car_gps_.longitude = msg->longitude;
    car_gps_.altitude = msg->altitude;
    publishNEDPosition();
}

void GPSToNED::droneGPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    drone_gps_.latitude = msg->latitude;
    drone_gps_.longitude = msg->longitude;
    drone_gps_.altitude = msg->altitude;
}

void GPSToNED::publishNEDPosition()
{
    if (car_gps_.latitude == 0.0 || drone_gps_.latitude == 0.0)
    {
        // Wait until both GPS positions are received
        return;
    }

    geometry_msgs::Point ned_position = convertGPSToNED(car_gps_, drone_gps_);
    ned_pub_.publish(ned_position);
}

geometry_msgs::Point GPSToNED::convertGPSToNED(const geographic_msgs::GeoPoint& car_gps, const geographic_msgs::GeoPoint& drone_gps)
{
    geodesy::UTMPoint car_utm(car_gps);
    geodesy::UTMPoint drone_utm(drone_gps);

    geometry_msgs::Point ned;
    ned.x = car_utm.northing - drone_utm.northing;
    ned.y = car_utm.easting - drone_utm.easting;
    ned.z = -(car_gps.altitude - drone_gps.altitude);

    return ned;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_to_ned_node");
    ros::NodeHandle nh("~");
    GPSToNED gps_to_ned(nh);
    ros::spin();
    return 0;
}
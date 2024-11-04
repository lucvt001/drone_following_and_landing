#ifndef GPS_TO_NED_H
#define GPS_TO_NED_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <geographic_msgs/GeoPoint.h>
#include <geodesy/utm.h>

class GPSToNED
{
public:
    GPSToNED(ros::NodeHandle& nh);

private:

    ros::NodeHandle nh_;
    ros::Subscriber car_gps_sub_;
    ros::Subscriber drone_gps_sub_;
    ros::Publisher ned_pub_;

    std::string car_gps_topic_;
    std::string drone_gps_topic_;
    std::string car_to_drone_ned_topic_;

    geographic_msgs::GeoPoint car_gps_;
    geographic_msgs::GeoPoint drone_gps_;

    void carGPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void droneGPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void publishNEDPosition();

    geometry_msgs::Point convertGPSToNED(const geographic_msgs::GeoPoint& car_gps, const geographic_msgs::GeoPoint& drone_gps);
};

#endif // GPS_TO_NED_H
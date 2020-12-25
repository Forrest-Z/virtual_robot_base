

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher pub_scan;
ros::Publisher pub_odom;

void callback_scan(const sensor_msgs::LaserScan& msg)
{
    sensor_msgs::LaserScan rmsg;
    rmsg = msg;
    rmsg.header.frame_id = "laser_front";
    pub_scan.publish(rmsg);
}

void callback_odom(const nav_msgs::Odometry& msg)
{
    nav_msgs::Odometry rmsg;
    rmsg = msg;
    rmsg.header.frame_id = "odom";
    pub_odom.publish(rmsg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cartographer_convert_topic");

    ros::NodeHandle node;

    auto sub_scan = node.subscribe("/front_scan", 2, &callback_scan);
    auto sub_odom = node.subscribe("/odom", 2, &callback_odom); 

    pub_scan = node.advertise<sensor_msgs::LaserScan>("/c_scan", 2);
    pub_odom = node.advertise<nav_msgs::Odometry>("/c_odom", 2);

    ros::spin();
    return 0;
}
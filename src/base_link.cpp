 /**  Created on: 2019年5月05日
  *      Author: liuquan
  * 
  * 模拟底盘节点
  * 傻乎乎的根据当前速度值来累积计算里程数据
  */


#include <thread>
#include <utility>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>
#include <cti_msgs/BaseSensors.h>
#include <std_msgs/Bool.h>

using namespace std::literals;

std::string baseFrameName, odomFrameName;
std::string cmdVelTopicName, odomTopicName, baseSensorTopicName;

/**
 * 傻乎乎的根据当前速度值来累积计算里程数据
 */

//简单的记录一下当前模拟底盘各种速度
static struct speed_info_s
{
    double vx {0}; /*正方向线速度*/
    double vy {0}; /*侧方向线速度*/
    double va {0}; /*角速度*/

    geometry_msgs::TwistStamped navi_cmd;

    bool clutch {true};

    //接收下发的数据命令，更新速度值
    void speed_cmd_vel_handler(const geometry_msgs::TwistStamped& value)
    {
        if (clutch)
        {
            vx = value.twist.linear.x;
            vy = value.twist.linear.y;
            va = value.twist.angular.z;
        }

        navi_cmd = value;
    }

    //接收test tool下发的数据命令, 更新速度值
    void speed_test_vel_handler(const geometry_msgs::TwistStamped& value)
    {
        vx = value.twist.linear.x;
        vy = value.twist.linear.y;
        va = value.twist.angular.z;
    }

    //简单的模拟一下各速度的衰减
    void update_speed_damping()
    {
        vx = std::fabs(vx) > 0.1 ? vx - (vx > 0 ? 1 : -1) * 0.1 : 0;
        vy = std::fabs(vy) > 0.1 ? vy - (vy > 0 ? 1 : -1) * 0.1 : 0;
        va = std::fabs(va) > 0.05 ? va - (va > 0 ? 1 : -1) * 0.05 : 0;
    }
    void clutch_handler(const std_msgs::Bool & msg)
    {
        clutch = msg.data;
    }
} speed_info;

//记录累计后的里程数据
static struct odom_info_s
{
    double x {0}, y {0}, yaw {0};
} odom_info;

//存储底盘的状态，主要存储主驱动轮是否已转向90°，后面用来决定是否横向更新里程数据
static struct base_state_s
{
    bool driving_wheel {false};
    void baseStateHandler(const cti_msgs::BaseSensors& value)
    {
        if (value.state_driving_wheel) driving_wheel = value.state_driving_wheel == 2;
    }
} base_state;

static void publishOdomInfo(ros::Publisher& publisher, tf::TransformBroadcaster& tfBroadcaster)
{
    tfBroadcaster.sendTransform(
        tf::StampedTransform(
                tf::Transform(tf::createQuaternionFromYaw(odom_info.yaw), tf::Vector3(odom_info.x, odom_info.y, 0))
                , ros::Time::now(), 
                odomFrameName, baseFrameName
            )
        );

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odomFrameName;
    odom.child_frame_id = baseFrameName;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_info.yaw);
    odom.pose.pose.position.x = odom_info.x;
    odom.pose.pose.position.y = odom_info.y;
    odom.pose.pose.position.z = 0;
    odom.twist.twist.linear.x = speed_info.vx;
    odom.twist.twist.angular.z = speed_info.va;

    odom.pose.covariance[0] = 0.1;
    odom.pose.covariance[7] = 0.1;
    odom.pose.covariance[14] = 10000000000.0;
    odom.pose.covariance[21] = 10000000000.0;
    odom.pose.covariance[28] = 10000000000.0;
    odom.pose.covariance[35] = 0.05;

    publisher.publish(odom);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "virtual_base_node");
    auto node = std::make_shared<ros::NodeHandle>("~");
    
    if (!node->getParam("odom_frame", odomFrameName)) odomFrameName = "odom";
    if (!node->getParam("base_frame", baseFrameName)) baseFrameName = "base_footprint";
    if (!node->getParam("base_sensor_topic", baseSensorTopicName)) baseSensorTopicName = "/robot_base/sensors";
    if (!node->getParam("odom_topic", odomTopicName)) odomTopicName = "/odom";
    if (!node->getParam("cmd_vel", cmdVelTopicName)) cmdVelTopicName = "/cmd_vel";

    auto cmdSub = node->subscribe(cmdVelTopicName, 32, &speed_info_s::speed_cmd_vel_handler, &speed_info);
    auto testoolCmdSub = node->subscribe("/cmd_vel_mux/output/test", 32, &speed_info_s::speed_test_vel_handler, &speed_info);

    auto stateSub = node->subscribe(baseSensorTopicName, 32, &base_state_s::baseStateHandler, &base_state);
    auto clutchSub = node->subscribe("/virtual_base_node/clutch", 1, &speed_info_s::clutch_handler, &speed_info);

    //里程数据累积计算线程
    std::thread([node]
    {
        uint32_t count = 0;
        auto odomPublisher = node->advertise<nav_msgs::Odometry>(odomTopicName, 32);
        auto cmdSendPublisher = node->advertise<geometry_msgs::TwistStamped>("/cmd_vel_send", 32);
        auto tfPublisher = tf::TransformBroadcaster();

        while (ros::ok())
        {
            if (count % 10 == 0) speed_info.update_speed_damping();              //每间隔200ms进行一次衰减
            if (count % 5 == 0) 
            {
                publishOdomInfo(odomPublisher, tfPublisher);     //每间隔100ms发布一次里程数据
                speed_info.navi_cmd.header.stamp = ros::Time::now();
                cmdSendPublisher.publish(speed_info.navi_cmd);
            }
            auto yaw = odom_info.yaw + (base_state.driving_wheel ? -M_PI_2 : 0);
            odom_info.yaw += speed_info.va / 50;
            odom_info.x += (speed_info.vx * std::cos(yaw) - speed_info.vy * std::sin(yaw)) / 50;
            odom_info.y += (speed_info.vx * std::sin(yaw) + speed_info.vy * std::cos(yaw)) / 50;
            std::this_thread::sleep_for(20ms);//以50Hz的频率进行更新
            count ++;
        }
    }).detach();

    ros::spin();
    
    return 0;
}

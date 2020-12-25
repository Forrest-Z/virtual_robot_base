 /**  Created on: 2019年5月14日
  *      Author: liuquan
  * 
  * 模拟装卸箱节点，替换原始的装卸箱节点和opencv二维码节点
  * 
  * 傻瓜式的实现方式：
  * 在收到装卸箱或进出充电桩时，就地开始平移0.8m后，发送操作完成
  * 再此过程中发布模拟的底盘传感器数据
  */

#include <thread>
#include <utility>
#include <memory>
#include <ros/ros.h>
#include <boost/optional.hpp>
#include <cti_msgs/BoxMountCmd.h>
#include <cti_msgs/BoxMountState.h>
#include <cti_msgs/BaseSensors.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <cti_msgs/BuildingRobotState.h>

using namespace std::literals;

/**
 */

std::string baseFrameName, mapFrameName;
std::string cmdVelTopicName, baseSensorTopicName, boxMountCmdTopicName, boxMountStatusTopicName;

enum class DockingProgress
{
    Idle,
    Mounting,
    Unmounting,
    Entering,
    Leaving
};

static struct docking_status_s
{
    DockingProgress progress;
    geometry_msgs::Pose origin;
    bool payload { false }, charge { false }, drivingWheel90 { false };

    std::weak_ptr<tf::TransformListener> tfListener;

    geometry_msgs::Pose getPose()
    {
        geometry_msgs::Pose pose;
        tf::StampedTransform st;
        do
        {
            try
            {
                tfListener.lock()->lookupTransform(mapFrameName, baseFrameName, ros::Time(), st);
                pose.orientation.x = st.getRotation().getX();
                pose.orientation.y = st.getRotation().getY();
                pose.orientation.z = st.getRotation().getZ();
                pose.orientation.w = st.getRotation().getW();
                pose.position.x = st.getOrigin().getX();
                pose.position.y = st.getOrigin().getY();
                pose.position.z = st.getOrigin().getZ();
            }
            catch(...)
            {
                try
                {
                    tfListener.lock()->waitForTransform(mapFrameName, baseFrameName, ros::Time(0), ros::Duration(2));
                }
                catch(...)
                {

                }
                std::this_thread::sleep_for(200s);
                continue;
            }
        } while(0);
        return pose;
    }

    void boxMountCmdHandler(const cti_msgs::BoxMountCmd& value)
    {
        if (value.cmd == cti_msgs::BoxMountCmd::BOX_CMD_MOUNT && !payload && progress == DockingProgress::Idle)
        {
            origin = getPose();
            progress = DockingProgress::Mounting;
        }
        else if (value.cmd == cti_msgs::BoxMountCmd::BOX_CMD_UNMOUNT && payload && progress == DockingProgress::Idle)
        {
            origin = getPose();
            progress = DockingProgress::Unmounting;
        }
        else if (value.cmd == cti_msgs::BoxMountCmd::CHARGE_CMD_ENTER && progress == DockingProgress::Idle)
        {
            origin = getPose();
            progress = DockingProgress::Entering;
        }
        else if (value.cmd == cti_msgs::BoxMountCmd::CHARGE_CMD_LEAVE && progress == DockingProgress::Idle)
        {
            origin = getPose();
            progress = DockingProgress::Leaving;
        }
    }
} docking_status;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "virtual_docking_node");
    auto node = ros::NodeHandle("~");
    auto listener = std::make_shared<tf::TransformListener>(node);
    docking_status.tfListener = listener;

    if (!node.getParam("map_frame", mapFrameName)) mapFrameName = "map";
    if (!node.getParam("base_frame", baseFrameName)) baseFrameName = "base_footprint";
    if (!node.getParam("base_sensor_topic", baseSensorTopicName)) baseSensorTopicName = "/robot_base/sensors";
    if (!node.getParam("boxmount_cmd_topic", boxMountCmdTopicName)) boxMountCmdTopicName = "/box_mount_cmd";
    if (!node.getParam("boxmount_status_topic", boxMountStatusTopicName)) boxMountStatusTopicName = "/box_mount_state";
    if (!node.getParam("cmd_vel_topic", cmdVelTopicName)) cmdVelTopicName = "/cmd_vel_mux/output/cmd_vel";

    auto boxMountCmdSub = node.subscribe(boxMountCmdTopicName, 32, &docking_status_s::boxMountCmdHandler, &docking_status);

    std::thread([&node]
    {
        auto cmdVelPub = node.advertise<geometry_msgs::TwistStamped>(cmdVelTopicName, 32);
        auto mountStatePub = node.advertise<cti_msgs::BoxMountState>(boxMountStatusTopicName, 16);
        auto baseSensorPub = node.advertise<cti_msgs::BaseSensors>(baseSensorTopicName, 16);
        while(ros::ok())
        {
            cti_msgs::BaseSensors sensor;
            sensor.header.stamp = ros::Time::now();
            sensor.header.frame_id = "virtual-sensor";
            sensor.state_charge = docking_status.charge;
            sensor.state_touch = docking_status.payload ? 2 : 1;
            sensor.state_driving_wheel = docking_status.drivingWheel90 ? 2 : 1;
            sensor.state_QR_led = docking_status.progress == DockingProgress::Mounting ? 2 : 1;
            sensor.state_plate = (sensor.state_carrying_box = (docking_status.payload && docking_status.progress == DockingProgress::Idle)) ? 2 : 1;

            baseSensorPub.publish(sensor);

            if (docking_status.progress != DockingProgress::Idle)
            {
                docking_status.drivingWheel90 = true;
                auto pose = docking_status.getPose();
                auto distance = std::hypot(pose.position.x - docking_status.origin.position.x, pose.position.y - docking_status.origin.position.y);

                geometry_msgs::TwistStamped cmd;
                cmd.header.stamp = ros::Time::now();
                cmd.header.frame_id = "virtual_docking";
                cmd.twist.linear.y = cmd.twist.linear.z = 0;
                cmd.twist.angular.x = cmd.twist.angular.y = cmd.twist.angular.z = 0;
                if (distance < 0.8)
                {
                    if ((docking_status.progress == DockingProgress::Mounting) || (docking_status.progress == DockingProgress::Entering))
                    {
                        cmd.twist.linear.x = 0.2;
                    }
                    else
                    {
                        cmd.twist.linear.x = -0.2;
                    }
                }
                else
                {
                    cmd.twist.linear.x = 0;
                }
                cmdVelPub.publish(cmd);

                if (distance > 0.8)
                {
                    cti_msgs::BoxMountState value;
                    switch (docking_status.progress)
                    {
                    case DockingProgress::Entering:
                        value.state = cti_msgs::BoxMountState::CHARGE_ENTER_SUCCESS;
                        docking_status.charge = true;
                        break;
                    case DockingProgress::Leaving:
                        value.state = cti_msgs::BoxMountState::CHARGE_LEAVE_SUCCESS;
                        docking_status.charge = false;
                        break;
                    case DockingProgress::Mounting:
                        value.state = cti_msgs::BoxMountState::BOX_MOUNT_SUCCESS;
                        docking_status.payload = true;
                        break;
                    case DockingProgress::Unmounting:
                        value.state = cti_msgs::BoxMountState::BOX_MOUNT_SUCCESS;
                        docking_status.payload = false;
                        break;
                    }
                    docking_status.progress = DockingProgress::Idle;
                    mountStatePub.publish(value);
                }
            }
            else
            {
                docking_status.drivingWheel90 = false;
            }

            std::this_thread::sleep_for(50ms);
        }
    }).detach();


    ros::spin();
    return 0;
}
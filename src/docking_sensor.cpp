 /**  Created on: 2019年5月08日
  *      Author: liuquan
  * 
  * 原计划是模拟底盘上，与装卸箱相关的所有传感器的联动反馈 （摄像头、接近开关、主驱动轮方向等各种底盘控制状态）
  * 现已放弃该方案（不能很好的理清楚各传感器之间的关系，联动模拟成本过高），改成直接模拟装卸箱节点的方式
  * 
  */

#include <utility>
#include <thread>
#include <boost/optional.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <cti_msgs/BaseSensors.h>
#include <cti_msgs/BaseControl.h>
#include <cti_msgs/DistinguishCmd.h>
#include <cti_msgs/DistinguishState.h>
#include <cti_msgs/BuildingRobotState.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
using namespace std::literals;

std::string mapFrameName, baseFrameName;
std::string qrStateTopicName, qrCmdReqTopicName, qrPoseTopicName;
std::string baseSensorTopicName, baseControlTopicName;

std::weak_ptr<ros::NodeHandle> nodeHandle;

geometry_msgs::Pose GetRobotPose()
{
    geometry_msgs::Pose robotPose;
    tf::TransformListener listener;
    
    while(true)
    {
        try
        {
            tf::StampedTransform t;
            listener.lookupTransform(mapFrameName, baseFrameName, ros::Time(0), t);
            robotPose.position.x = t.getOrigin().x();
            robotPose.position.y = t.getOrigin().y();
            robotPose.position.z = t.getOrigin().z();
            robotPose.orientation.x = t.getRotation().x();
            robotPose.orientation.y = t.getRotation().y();
            robotPose.orientation.z = t.getRotation().z();
            robotPose.orientation.w = t.getRotation().w();
            break;
        }
        catch(...)
        {
            try
            {
                listener.waitForTransform(mapFrameName, baseFrameName, ros::Time(0), ros::Duration(2));
            }
            catch(...)
            {
                std::this_thread::sleep_for(1s);
            }
        }
    }
    return robotPose;
}

static struct base_state_s
{
    struct
    {
        struct
        {
            int8_t state {1}, to {0};
            time_t timestamp {0};
        } driving_wheel, plate;
        int8_t QR_led {1}, touch {0}, carrying_box {1}, proximity_switch {2}, charge {2};
    } state;
    
    boost::optional<geometry_msgs::Pose> origin, robotPose, targetHive;
    
    void baseControlHandler(const cti_msgs::BaseControl& value)
    {
        if (state.driving_wheel.state && value.ctrl_driving_wheel && value.ctrl_driving_wheel != state.driving_wheel.state)
        {
            state.driving_wheel.state = 0;
            state.driving_wheel.to = value.ctrl_driving_wheel;
            state.driving_wheel.timestamp = ::time(nullptr) + 2;
        }
        if (state.plate.state && value.ctrl_plate && value.ctrl_plate != state.plate.state)
        {
            state.plate.state = 0;
            state.plate.to = value.ctrl_plate;
            state.plate.timestamp = ::time(nullptr) + 3;
        }
        if (value.ctrl_touch) state.touch = value.ctrl_touch;
        if (value.ctrl_QR_led) state.QR_led = value.ctrl_QR_led;
    }
    
    void boxcmdReqHandler(const cti_msgs::DistinguishCmd& value)
    {
        static bool startFindQCode = false;
        static std::shared_ptr<std::thread> thread = nullptr;
        startFindQCode = value.command == cti_msgs::DistinguishCmd::CMD_START_FIND_QR;
        if (startFindQCode && !thread)
        {
            thread = std::make_shared<std::thread>([this, node = nodeHandle.lock()]
            {
                targetHive = origin = robotPose = GetRobotPose();
                targetHive->position.y -= 0.8;
                //targetHive->position.x += 1;
                
                auto posePub = node->advertise<geometry_msgs::Pose>(qrPoseTopicName, 32);
                auto statePub = node->advertise<cti_msgs::DistinguishState>(qrStateTopicName, 32);
                
                while(ros::ok() && startFindQCode)
                {
                    robotPose = GetRobotPose();
                    
                    if (targetHive)
                    {
                        auto moved = std::hypot(std::fabs(origin->position.x - robotPose->position.x), std::fabs(origin->position.y - robotPose->position.y));
                        auto distance = std::hypot(std::fabs(origin->position.x - targetHive->position.x), std::fabs(origin->position.y - targetHive->position.y));
    
                        geometry_msgs::Pose pose;
                        //pose.position.x = robotPose->position.y - targetHive->position.y;
                        //pose.position.y = robotPose->position.x - targetHive->position.x;
                        pose.position.x = targetHive->position.y - robotPose->position.y;
                        pose.position.y = targetHive->position.x - robotPose->position.x;
                        pose.position.z = robotPose->position.z;
                        auto yaw = std::atan(std::fabs(pose.position.y) / std::fabs(pose.position.x));
                        //if (pose.position.x < 0) yaw = M_PI - yaw;
                        if (pose.position.y < 0) yaw *= -1;
                        auto orientation = tf::createQuaternionFromYaw(yaw);
                        pose.orientation.x = orientation.x();
                        pose.orientation.y = orientation.y();
                        pose.orientation.z = orientation.z();
                        pose.orientation.w = orientation.w();

                        std::cout << pose.position.x << ", " << pose.position.y << ", " << yaw << std::endl;
                        
                        cti_msgs::DistinguishState qr_state;
                        
                        qr_state.state = moved / distance < 0.5 ? cti_msgs::DistinguishState::STATE_DISTINGUISH_MOVING : cti_msgs::DistinguishState::STATE_DISTINGUISH_TOO_CLOSE;
                        statePub.publish(qr_state);
                        posePub.publish(pose);
    
                        state.proximity_switch = static_cast<int8_t>(moved / distance > 0.6 ? 1 : 2);
                    }
                    
                    std::this_thread::sleep_for(50ms);
                }
    
                posePub.shutdown();
                statePub.shutdown();
                
                origin.reset();
                robotPose.reset();
                targetHive.reset();
            });
        }
    }
} base_state;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "virutal_docking_sensor_node");
    
    auto node = std::make_shared<ros::NodeHandle>("~");
    nodeHandle = node;
    
    if (!node->getParam("map_frame", mapFrameName)) mapFrameName = "map";
    if (!node->getParam("base_frame", baseFrameName)) baseFrameName = "base_link";
    if (!node->getParam("qr_request_topic", qrCmdReqTopicName)) qrCmdReqTopicName = "/distinguish_cmd";
    if (!node->getParam("qr_state_topic", qrStateTopicName)) qrStateTopicName = "/distinguish_state";
    if (!node->getParam("qr_pose_topic", qrPoseTopicName)) qrPoseTopicName = "/qrcode_rela_pose";
    if (!node->getParam("base_sensor_topic", baseSensorTopicName)) baseSensorTopicName = "/robot_base/sensors";
    if (!node->getParam("base_ctrl_topic", baseControlTopicName)) baseControlTopicName = "/robot_docking/control";
    
    std::thread([node]
    {
        
        auto controlSubscriber = node->subscribe(baseControlTopicName, 32, &base_state_s::baseControlHandler, &base_state);
        auto sensorPublisher = node->advertise<cti_msgs::BaseSensors>(baseSensorTopicName, 32);
        
        auto qrcmdreqSubscriber = node->subscribe(qrCmdReqTopicName, 32, &base_state_s::boxcmdReqHandler, &base_state);
        
        while(ros::ok())
        {
            if (base_state.state.driving_wheel.timestamp && time(nullptr) > base_state.state.driving_wheel.timestamp)
            {
                base_state.state.driving_wheel.timestamp = 0;
                base_state.state.driving_wheel.state = base_state.state.driving_wheel.to;
            }
            if (base_state.state.plate.timestamp && time(nullptr) > base_state.state.plate.timestamp)
            {
                base_state.state.plate.timestamp = 0;
                if ((base_state.state.plate.state = base_state.state.plate.to) == 1)
                {
                    base_state.state.proximity_switch = 1;
                }
            }
            
            cti_msgs::BaseSensors state;
            state.header.frame_id = "v_base_sensor_state";
            state.header.stamp = ros::Time::now();
            state.state_plate = static_cast<unsigned char>(base_state.state.plate.state);
            state.state_QR_led = static_cast<unsigned char>(base_state.state.QR_led);
            state.state_touch = static_cast<unsigned char>(base_state.state.touch);
            state.state_charge = static_cast<unsigned char>(base_state.state.charge);
            state.state_driving_wheel = static_cast<unsigned char>(base_state.state.driving_wheel.state);
            state.state_proximity_switch = static_cast<unsigned char>(base_state.state.proximity_switch);
            
            if((state.state_plate == 2) && (state.state_proximity_switch == 1))
            {
                state.state_carrying_box = 1;
            }
            else
            {
                state.state_carrying_box = 0;
            }
            sensorPublisher.publish(state);
            
            std::this_thread::sleep_for(500ms);
        }
    }).detach();
    
    ros::spin();
    
    return 0;
}
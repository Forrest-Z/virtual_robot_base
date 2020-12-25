 /**  Created on: 2019年5月08日
  *      Author: liuquan
  * 
  * 模拟梯控节点
  * 
  * 简单粗暴的实现方式，在收到呼梯指令后，开线程①，该线程在启动的3s后发送呼梯成功
  * 在机器人进入电梯内等待状态时，开启线程②，该线程在启动的15s后发送到达/门已开的指令
  */

#include <ros/ros.h>
#include <utility>
#include <thread>
#include <cti_msgs/LoraCmd.h>
#include <cti_msgs/LoraMsg.h>
#include <cti_msgs/LiftState.h>
#include <cti_msgs/ElevatorCmd.h>
#include <cti_msgs/PassagewayCmd.h>
#include <cti_msgs/PassagewayState.h>
#include <cti_msgs/BuildingRobotState.h>

ros::Publisher liftStatePublisher;
ros::Publisher GateStatePublisher;
ros::Publisher LoraStatePublisher;

using namespace std::literals;

bool gTh = false;
double gOutsideDoorWaitTime = 0.0;
double gInsideDoorWaitTime = 0.0;

static void ElevatorCmdHandler(const cti_msgs::ElevatorCmd& value)
{
    if (value.cmd == cti_msgs::ElevatorCmd::CMD_CALL_ELEVATOR)
    {
        //线程①
        std::thread([]{
            {
                ros::Duration(gOutsideDoorWaitTime).sleep();
                cti_msgs::LiftState s;
                s.state = cti_msgs::LiftState::LIFTSTATE_OPENDOOR_SUCCESS;
                liftStatePublisher.publish(s);
            }}).detach();
    }
}

static void RobotStateHandler(const cti_msgs::BuildingRobotState& value)
{
    if ((ros::Time::now() - value.timestamp).toSec() > 5.0)
    {
        return;
    }

    static int robotState = -1;
    if (robotState != value.state)
    {
        robotState = value.state;
        if ((robotState == cti_msgs::BuildingRobotState::STATE_ALLINLIFT_AND_STILLMOVINGIN)
            || (robotState == cti_msgs::BuildingRobotState::STATE_WAITTING_LIFE_INSIDE))
        {
            if (!gTh)
            {
                //线程②
                std::thread([&gTh]{
                    {
                        gTh = true;
                        ros::Duration(gInsideDoorWaitTime).sleep();
                        cti_msgs::LiftState s;
                        s.state = cti_msgs::LiftState::LIFTSTATE_OPENDOOR_SUCCESS;
                        liftStatePublisher.publish(s);
                        gTh = false;
                    }}).detach();
            }
        }
    }
}

static void GateCmdHandler(const cti_msgs::PassagewayCmd& value)
{
    if (value.cmd == cti_msgs::PassagewayCmd::PASSAGEWAY_CMD_APPLY)
    {
        std::thread([]{
            {
                std::this_thread::sleep_for(5s);
                cti_msgs::PassagewayState s;
                s.state = cti_msgs::PassagewayState::PASSAGEWAY_STATE_APPLY_SUCCESS;
                GateStatePublisher.publish(s);
                
                // std::cout << "PassagewayState::PASSAGEWAY_STATE_APPLY_SUCCESS" << std::endl;
            }}).detach();
    }
}

static void LoraCmdHandler(const cti_msgs::LoraCmd& cmd)
{
    cti_msgs::LoraMsg msg;

    if (cmd.cmd == cti_msgs::LoraCmd::CMD_APPLY_LORA_DEVICE)
    {
        msg.state = cti_msgs::LoraMsg::STATE_APPLY_SUCCESS;
        msg.source = cmd.source;
        LoraStatePublisher.publish(msg);
    }
    else if (cmd.cmd == cti_msgs::LoraCmd::CMD_SEND_MSG)
    {
        msg.state = cti_msgs::LoraMsg::STATE_RECEVICE_DATA;
        msg.source = cmd.source;
        msg.data = cmd.info;
        LoraStatePublisher.publish(msg);
    }

    return;
}

int main(int argc, char** argv)
{
    std::string liftTopicName, stateTopicName, elevatorCmdTopicName;
    ros::init(argc, argv, "virtual_elevator_ctrl_node");
    
    auto node = std::make_shared<ros::NodeHandle>("~");
    if (!node->getParam("lift_topic", liftTopicName)) liftTopicName = "/lift_state";
    if (!node->getParam("robot_state_topic", stateTopicName)) stateTopicName = "/robot_state";
    if (!node->getParam("elevator_cmd_topic", elevatorCmdTopicName)) elevatorCmdTopicName = "/elevator_cmd";
    if (!node->getParam("outside_door_wait_time", gOutsideDoorWaitTime)) gOutsideDoorWaitTime = 10.0;
    if (!node->getParam("inside_door_wait_time", gInsideDoorWaitTime)) gInsideDoorWaitTime = 10.0;
    ROS_INFO("Vir elevator: out:%lf, in:%lf", gOutsideDoorWaitTime, gInsideDoorWaitTime);
    liftStatePublisher = node->advertise<cti_msgs::LiftState>(liftTopicName, 32);
    GateStatePublisher = node->advertise<cti_msgs::PassagewayState>("/passageway_state", 32);
    LoraStatePublisher = node->advertise<cti_msgs::LoraMsg>("/lora_msg", 32);
    auto robotStateSub = node->subscribe(stateTopicName, 32, RobotStateHandler);
    auto elevatorCmdSub = node->subscribe(elevatorCmdTopicName, 32, ElevatorCmdHandler);
    auto gateCmdSub = node->subscribe("/passageway_cmd", 32, GateCmdHandler);
    auto loraCmdSub = node->subscribe("/lora_cmd", 32, LoraCmdHandler);
    ros::spin();
    return 0;
}
#include <ros/ros.h>
#include <utility>
#include <thread>
#include <cti_msgs/LiftState.h>
#include <cti_msgs/ElevatorCmd.h>
#include <cti_msgs/PassagewayCmd.h>
#include <cti_msgs/PassagewayState.h>
#include <cti_msgs/BuildingRobotState.h>

class ShowElevatorCtrl
{
    public:

        ShowElevatorCtrl(std::shared_ptr<ros::NodeHandle> n, std::shared_ptr<ros::NodeHandle> pn)
        {
            if (!pn->getParam("lift_topic", mLiftTopicName)) mLiftTopicName = "/lift_state";
            if (!pn->getParam("robot_state_topic", mStateTopicName)) mStateTopicName = "/robot_state";
            if (!pn->getParam("elevator_cmd_topic", mElevatorCmdTopicName)) mElevatorCmdTopicName = "/elevator_cmd";
            if (!pn->getParam("outside_door_wait_time", gOutsideDoorWaitTime)) gOutsideDoorWaitTime = 10.0;
            if (!pn->getParam("inside_door_wait_time", gInsideDoorWaitTime)) gInsideDoorWaitTime = 10.0;
            ROS_INFO("Vir elevator: out:%lf, in:%lf", gOutsideDoorWaitTime, gInsideDoorWaitTime);
            mPubliftState = pn->advertise<cti_msgs::LiftState>(mLiftTopicName, 32);
            mPubGateState = pn->advertise<cti_msgs::PassagewayState>("/passageway_state", 32);
            // mSubRobotState = pn->subscribe(mStateTopicName, 32, &ShowElevatorCtrl::RobotStateHandler, this);
            mSubElevatorCmd = pn->subscribe(mElevatorCmdTopicName, 32, &ShowElevatorCtrl::ElevatorCmdHandler, this);
            mSubGateCmd = pn->subscribe("/passageway_cmd", 32, &ShowElevatorCtrl::GateCmdHandler, this);
        };

        virtual ~ ShowElevatorCtrl(){};

    private:

        ros::Publisher mPubliftState, mPubGateState;
        ros::Subscriber mSubRobotState, mSubElevatorCmd, mSubGateCmd;
        ros::Timer mOpenDoor;

        std::string mLiftTopicName, mStateTopicName, mElevatorCmdTopicName;

        double gOutsideDoorWaitTime = 0.0;
        double gInsideDoorWaitTime = 0.0;

        void ElevatorCmdHandler(const cti_msgs::ElevatorCmd& value)
        {

            if (value.cmd == cti_msgs::ElevatorCmd::CMD_CALL_ELEVATOR)
            {
                // std::thread([this]{
                // {
                // }}).detach();
                //发送开门命令
                //启动一个定时器,检查门是否打开
                //门打开后,启动一个定时器,定时发开门命令并检查门是否打开
            }
            else if (value.cmd == cti_msgs::ElevatorCmd::CMD_CLOSE_DOOR)
            {
                //发送关门命令
            }
            else if (value.cmd == cti_msgs::ElevatorCmd::CMD_CANCEL_TASK)
            {
                //暂无处理
            }
            else
            {
                //无处理
            }
            return;
        }


        void GateCmdHandler(const cti_msgs::PassagewayCmd& value)
        {
            // if (value.cmd == cti_msgs::PassagewayCmd::PASSAGEWAY_CMD_APPLY)
            // {
            //     std::thread([]{
            //         {
            //             std::this_thread::sleep_for(5s);
            //             cti_msgs::PassagewayState s;
            //             s.state = cti_msgs::PassagewayState::PASSAGEWAY_STATE_APPLY_SUCCESS;
            //             GateStatePublisher.publish(s);
                        
            //             // std::cout << "PassagewayState::PASSAGEWAY_STATE_APPLY_SUCCESS" << std::endl;
            //         }}).detach();
            // }
        }
};

int main(int argc, char** argv)
{
    std::string liftTopicName, stateTopicName, elevatorCmdTopicName;
    ros::init(argc, argv, "virtual_show_elevator_ctrl_node");
    
    auto n = std::make_shared<ros::NodeHandle>();
    auto pn = std::make_shared<ros::NodeHandle>("~");

    ShowElevatorCtrl show(n, pn);

    ros::spin();
    return 0;
}

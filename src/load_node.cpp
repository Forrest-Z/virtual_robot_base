#include <thread>
#include <utility>

#include <ros/ros.h>
#include <cti_msgs/BoxMountCmd.h>
#include <cti_msgs/BoxMountState.h>
#include <cti_msgs/BaseSensors.h>
#include <geometry_msgs/TwistStamped.h>

using namespace std::literals;

typedef enum {
    SL_IDEL = 0,
    SL_ENTER = 1,
    SL_UP = 2,
    SL_DOWM = 3,
    SL_LEAVE = 4,
    SL_CHARGE = 5
}STATUS_LIST;

STATUS_LIST status = SL_IDEL;

#define NO_CMD 99;
unsigned int cmd = NO_CMD; 

unsigned int carrybox = 0;

void boxCmdHandler(const cti_msgs::BoxMountCmd& msg)
{
    if (status == SL_IDEL)
    {
        cmd = msg.cmd;
    }
    else if (cmd == cti_msgs::BoxMountCmd::BOX_CMD_MOUNT
            && msg.cmd == cti_msgs::BoxMountCmd::BOX_CMD_RECOVERY_MOUNT)
    {
        cmd = msg.cmd;
    }
    else if (cmd == cti_msgs::BoxMountCmd::BOX_CMD_UNMOUNT
            && msg.cmd == cti_msgs::BoxMountCmd::BOX_CMD_RECOVERY_UNMOUNT)
    {
        cmd = msg.cmd;
    }
    else if (cmd == cti_msgs::BoxMountCmd::TEST_CMD_UNMOUNT
            && msg.cmd == cti_msgs::BoxMountCmd::BOX_CMD_RECOVERY_UNMOUNT)
    {
        cmd = msg.cmd;
    }
    else if ((cmd == cti_msgs::BoxMountCmd::CHARGE_CMD_ENTER 
           || cmd == cti_msgs::BoxMountCmd::CHARGE_CMD_CONTINUE_WITHBOX
           || cmd == cti_msgs::BoxMountCmd::CHARGE_CMD_CONTINUE_WITHOUTBOX)
              && msg.cmd == cti_msgs::BoxMountCmd::CHARGE_CMD_LEAVE)
    {
        cmd = msg.cmd;
    }
    else
    {
        /* code */
    }
    
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "virtual_load_node");

    auto node = ros::NodeHandle("~");

    auto subCmd = node.subscribe("/box_mount_cmd", 1, &boxCmdHandler);

    auto pubRobotSensors = node.advertise<cti_msgs::BaseSensors>("/robot_base/sensors", 1);

    ros::Timer timer = node.createTimer(ros::Duration(1.0), 
                    [carrybox, &pubRobotSensors]
                    (const ros::TimerEvent &event)
                        {
                            cti_msgs::BaseSensors msg;
                            msg.state_carrying_box = carrybox;
                            msg.state_charge = -200;
                            pubRobotSensors.publish(msg);
                            // ROS_ERROR("TIMER %d", carrybox);
                            return;
                        }
                    );

    std::thread([&node]{

        auto pubLiftState = node.advertise<cti_msgs::BoxMountState>("/box_mount_state", 1);
        auto pubCmdVel = node.advertise<geometry_msgs::TwistStamped>("/cmd_vel_mux/output/cmd_vel", 1);

        int near_switch_times, lift_switch_times, drop_switch_times, leav_switch_times;
        int default_near, default_lift, default_frop, default_leav;

        node.param("near_switch_times", default_near, 200);
        node.param("lift_switch_times", default_lift, 200);
        node.param("drop_switch_times", default_frop, 200);
        node.param("leav_switch_times", default_leav, 200);

        //ros::Duration(11*60).sleep();
        
        while (ros::ok())
        {
            //根据上层命令初始化/重置状态
            switch (status)
            {
                case SL_IDEL:
                {
                    switch (cmd)
                    {
                        case cti_msgs::BoxMountCmd::BOX_CMD_MOUNT :
                        case cti_msgs::BoxMountCmd::TEST_CMD_UNMOUNT :
                        {
                            near_switch_times = default_near;
                            lift_switch_times = default_lift;
                            drop_switch_times = default_frop;
                            leav_switch_times = default_leav;

                            status = SL_ENTER;
                            break;
                        }
                        case cti_msgs::BoxMountCmd::BOX_CMD_UNMOUNT :
                        {
                            // near_switch_times = default_near;
                            lift_switch_times = default_lift;
                            drop_switch_times = default_frop;
                            leav_switch_times = default_leav;

                            status = SL_DOWM;
                            break;
                        }
                        case cti_msgs::BoxMountCmd::CHARGE_CMD_ENTER :
                        {
                            near_switch_times = default_near;
                            drop_switch_times = default_frop;
                            lift_switch_times = default_lift;
                            leav_switch_times = default_leav;
                            status = SL_ENTER;
                            break;
                        }
                        case cti_msgs::BoxMountCmd::CHARGE_CMD_CONTINUE_WITHBOX :
                        case cti_msgs::BoxMountCmd::CHARGE_CMD_CONTINUE_WITHOUTBOX :
                        {
                            lift_switch_times = default_lift;
                            leav_switch_times = default_leav;
                            status = SL_CHARGE;
                            break;
                        }
                        default:
                        {
                            break;
                        }
                    }
                    
                    break;
                }
                case SL_ENTER:
                {
                    switch (cmd)
                    {
                        case cti_msgs::BoxMountCmd::BOX_CMD_MOUNT :
                        {
                            if (near_switch_times == 0)
                            {
                                status = SL_UP;
                            }
                            break;
                        }
                        case cti_msgs::BoxMountCmd::BOX_CMD_UNMOUNT :
                        case cti_msgs::BoxMountCmd::TEST_CMD_UNMOUNT :
                        {
                            if (near_switch_times == 0)
                            {
                                status = SL_DOWM;
                            }
                            break;
                        }
                        case cti_msgs::BoxMountCmd::BOX_CMD_RECOVERY_MOUNT :
                        case cti_msgs::BoxMountCmd::BOX_CMD_RECOVERY_UNMOUNT :
                        {
                            leav_switch_times = default_near - near_switch_times;
                            status = SL_LEAVE;
                            break;
                        }
                        case cti_msgs::BoxMountCmd::CHARGE_CMD_ENTER :
                        {
                            if (near_switch_times == 0)
                            {
                                status = SL_DOWM;
                            }
                        }
                        default :
                        {
                            break;
                        }
                    }

                    break;
                }
                case SL_UP:
                {
                    switch (cmd)
                    {
                        case cti_msgs::BoxMountCmd::BOX_CMD_MOUNT :
                        {
                            if (lift_switch_times == 0)
                            {
                                carrybox = 1;
                                status = SL_LEAVE;
                                cti_msgs::BoxMountState msg;
                                msg.state = cti_msgs::BoxMountState::BOX_CLOSE_TO_SUCCESS;
                                msg.timestamp = ros::Time::now();
                                pubLiftState.publish(msg);
                            }
                            break;
                        }
                        case cti_msgs::BoxMountCmd::CHARGE_CMD_LEAVE :
                        {
                            if (lift_switch_times == 0)
                            {
                                status = SL_LEAVE;
                            }
                            break;
                        }
                        default:
                        {
                            break;
                        }
                    }
                    
                    break;
                }
                case SL_DOWM:
                {
                    switch (cmd)
                    {
                        case cti_msgs::BoxMountCmd::BOX_CMD_UNMOUNT :
                        case cti_msgs::BoxMountCmd::TEST_CMD_UNMOUNT :
                        {
                            if (drop_switch_times == 0)
                            {
                                carrybox = 0;
                                status = SL_LEAVE;
                            }
                            break;
                        }
                        case cti_msgs::BoxMountCmd::CHARGE_CMD_ENTER :
                        {
                            if (drop_switch_times == 0)
                            {
                                carrybox = 0;
                                status = SL_CHARGE;
                                cti_msgs::BoxMountState msg;
                                msg.state = cti_msgs::BoxMountState::CHARGE_ENTER_SUCCESS;
                                msg.timestamp = ros::Time::now();
                                pubLiftState.publish(msg);
                            }
                            break;
                        }
                        default:
                        {
                            break;
                        }
                    }
                    break;
                }
                case SL_CHARGE:
                {
                    switch (cmd)
                    {
                        case cti_msgs::BoxMountCmd::CHARGE_CMD_ENTER :
                        {
                            //do nothing , wait stop cmd
                            break;
                        }
                        case cti_msgs::BoxMountCmd::CHARGE_CMD_LEAVE :
                        {
                            status = SL_UP;
                            break;
                        }
                        default:
                        {
                            break;
                        }
                    }
                }
                case SL_LEAVE:
                {
                    switch (cmd)
                    {
                        case cti_msgs::BoxMountCmd::BOX_CMD_MOUNT :
                        case cti_msgs::BoxMountCmd::BOX_CMD_UNMOUNT :
                        case cti_msgs::BoxMountCmd::TEST_CMD_UNMOUNT :
                        {
                            if (leav_switch_times == 0)
                            {
                                status = SL_IDEL;
                                cti_msgs::BoxMountState msg;
                                msg.state = cti_msgs::BoxMountState::BOX_MOUNT_SUCCESS;
                                msg.timestamp = ros::Time::now();
                                // ros::Duration(5.0).sleep();
                                pubLiftState.publish(msg);
                                cmd = NO_CMD;
                            }
                            break;
                        }
                        case cti_msgs::BoxMountCmd::BOX_CMD_RECOVERY_MOUNT :
                        case cti_msgs::BoxMountCmd::BOX_CMD_RECOVERY_UNMOUNT :
                        {
                            if (leav_switch_times == 0)
                            {
                                status = SL_IDEL;
                                cti_msgs::BoxMountState msg;
                                msg.state = cti_msgs::BoxMountState::BOX_RECOVERY_SUCCESS;
                                msg.timestamp = ros::Time::now();
                                pubLiftState.publish(msg);
                                cmd = NO_CMD;
                            }
                            break;
                        }
                        case cti_msgs::BoxMountCmd::CHARGE_CMD_LEAVE :
                        {
                            if (leav_switch_times == 0)
                            {
                                cti_msgs::BoxMountState msg;
                                msg.state = cti_msgs::BoxMountState::CHARGE_LEAVE_SUCCESS;
                                msg.timestamp = ros::Time::now();
                                pubLiftState.publish(msg);
                                status = SL_IDEL;
                                cmd = NO_CMD;
                            }
                            break;
                        }
                        default :
                        {
                            break;
                        }
                    }
                    
                    break;
                }
                default:
                {
                    break;
                }
            }

            //模拟装箱过程中的传感器命令
            switch (status)
            {
                case SL_IDEL:
                {
                    break;
                }
                case SL_ENTER:
                {
                    geometry_msgs::TwistStamped cmd;
                    cmd.header.stamp = ros::Time::now();
                    cmd.header.frame_id = "virtual_load";
                    cmd.twist.linear.x = cmd.twist.linear.z = 0;
                    cmd.twist.angular.x = cmd.twist.angular.y = cmd.twist.angular.z = 0;
                    cmd.twist.linear.y = -0.1;
                    pubCmdVel.publish(cmd);

                    near_switch_times --;
                    break;
                }
                case SL_UP:
                {
                    lift_switch_times --;
                    break;
                }
                case SL_DOWM:
                {
                    drop_switch_times --;
                    break;
                }
                case SL_LEAVE:
                {
                    geometry_msgs::TwistStamped cmd;
                    cmd.header.stamp = ros::Time::now();
                    cmd.header.frame_id = "virtual_load";
                    cmd.twist.linear.x = cmd.twist.linear.z = 0;
                    cmd.twist.angular.x = cmd.twist.angular.y = cmd.twist.angular.z = 0;
                    cmd.twist.linear.y = 0.1;
                    pubCmdVel.publish(cmd);

                    leav_switch_times --;
                    break;
                }
                default:
                {
                    break;
                }
            }

            std::this_thread::sleep_for(50ms);
        }
        
    }).detach();

    ros::spin();
    return 0;
}
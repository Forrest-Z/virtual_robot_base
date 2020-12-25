/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @Date: 2019-08-15 11:18:25
 * @LastEditTime: 2020-05-11 10:08:03
 * @LastEditors: Please set LastEditors
 */
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <utility>
#include <thread>
// #include <queue>
#include <deque>
#include <mutex>
#include <string>
#include <condition_variable>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"

#include "cti_msgs/BuildingRobotState.h"

#include "ros/ros.h"
#include "yaml-cpp/yaml.h"

using namespace std::literals;

template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}

class CmdBaseInfo
{
public:
    int type;
    int action;

    CmdBaseInfo(){};
    virtual ~ CmdBaseInfo(){};

    virtual std::string getBuilding(){};
    virtual int getLevel(){};
    virtual geometry_msgs::Pose getPose(){};
    virtual std::string getQR(){};
    virtual int getSleepTime(){};
};

class SleepCmdInfo : public CmdBaseInfo
{
public:
    SleepCmdInfo(){};
    virtual ~ SleepCmdInfo(){};

    SleepCmdInfo(YAML::Node node)
    {
        node["type"] >> type;
        node["sleep_time"] >> sleep_time;
    };

    int getSleepTime(){ return sleep_time;}

private:
    int sleep_time;
};


class HiveCmdInfo : public CmdBaseInfo
{
public:
    HiveCmdInfo(){};
    virtual ~ HiveCmdInfo(){};

    HiveCmdInfo(YAML::Node node)
    {
        node["action"] >> action;
        node["type"] >> type;
        node["qr"] >> qr;
    };

    std::string getQR(){ return qr;}

private:
    std::string qr;
};

class NaviCmdInfo : public CmdBaseInfo
{
public:
    NaviCmdInfo(){};
    virtual ~ NaviCmdInfo(){};

    NaviCmdInfo(YAML::Node node)
    {
        node["action"] >> action;
        node["type"] >> type;
        node["building"] >> building;
        node["level"] >> level;
        node["pose"]["x"] >> pose.position.x;
        node["pose"]["y"] >> pose.position.y;
        node["pose"]["z"] >> pose.orientation.z;
        node["pose"]["w"] >> pose.orientation.w;

        pose.position.z = pose.orientation.x = pose.orientation.y = 0.0;
    }

    std::string getBuilding(){ return building;}
    int getLevel(){ return level;}
    geometry_msgs::Pose getPose(){ return pose;}

private:
    std::string building;
    int level;
    geometry_msgs::Pose pose;
};

class ITask {
public:

    ITask(){};
    virtual ~ITask(){};

    virtual void initTask(ros::Publisher p, std::shared_ptr<CmdBaseInfo> cbi)
    {
        ROS_ERROR("-ITask-init task--");
    };

    virtual void pubTask()
    {
        ROS_ERROR("-ITask-pubTask task--");
    };
};

class HiveTask final : public ITask {

public:
    HiveTask(){};
    virtual~ HiveTask(){};

    void initTask(ros::Publisher p, std::shared_ptr<CmdBaseInfo> cbi){
        ROS_ERROR("-HiveTask-init task--");
        mP = p;
        mMSG.qr = cbi->getQR();
        mMSG.action = cbi->action;
    }

    void pubTask()
    {
        ROS_ERROR("-pub hive task -%d-", mMSG.action);
        mP.publish(mMSG);
    }

private:
    
    ros::Publisher mP;
};

class SleepTask final : public ITask {

public:
    SleepTask(){};
    virtual~ SleepTask(){};

    void initTask(ros::Publisher p, std::shared_ptr<CmdBaseInfo> cbi){
        ROS_ERROR("-SleepTask-init task--");
        mSleep_time = cbi->getSleepTime();
    }

    void pubTask()
    {
        ROS_ERROR("-pub sleep task -%d s-", mSleep_time);
        ros::Duration(mSleep_time).sleep();
    }

private:
   int mSleep_time;
};

class NaviTask final : public ITask {

public:
    NaviTask(){};
    virtual~ NaviTask(){};

    void initTask(ros::Publisher p, std::shared_ptr<CmdBaseInfo> cbi){
        ROS_ERROR("-NaviTask-init task--");
        mP = p;
        mMSG.action = cbi->action;
        mMSG.building = cbi->getBuilding();
        mMSG.level = cbi->getLevel();
        mMSG.coordinates = cbi->getPose();
        ROS_ERROR("-NaviTask-init task-%s, %s, %lf, %lf-", 
            mMSG.building.c_str(), 
            mMSG.level.c_str(),
            mMSG.coordinates.position.x,
            mMSG.coordinates.position.y);
    }

    void pubTask()
    {
        ROS_ERROR("-pub navi task -%d-", mMSG.action);
        mP.publish(mMSG);
    }

private:
    cti_msgs::NaviRequest mMSG;
    
    ros::Publisher mP;
};

class CmdSet
{
public:
    CmdSet(){};
    virtual ~ CmdSet(){};

    CmdSet(std::string cmdnum, YAML::Node node){

        mCmdSetName = cmdnum;
        node["cmdnum"] >> mCmdNum;

        for(int i=1; i<=mCmdNum; i++)
        {
            int type;
            // char tmp[2] = {0};
            // tmp[0] = 48 + i;
            // std::string s(tmp);
            std::string s = std::to_string(i);
            node[s.c_str()]["type"] >> type;
            ROS_ERROR("%s : type : %d", s.c_str(), type);
            switch (type)
            {
                case 0: //navi
                {
                    auto nc = std::make_shared<NaviCmdInfo>(node[s.c_str()]);
                    mVeCmdSetInfo.push_back(nc);
                    break;
                }
                case 1: //hive
                {
                    auto hc = std::make_shared<HiveCmdInfo>(node[s.c_str()]);
                    mVeCmdSetInfo.push_back(hc);
                    break;
                }
                case 3: //sleep
                {
                    auto sc = std::make_shared<SleepCmdInfo>(node[s.c_str()]);
                    mVeCmdSetInfo.push_back(sc);
                    break;
                }
                default:
                {
                    break;
                }
            }
        } 
    };

    std::string mCmdSetName;
    std::vector<std::shared_ptr<CmdBaseInfo>> mVeCmdSetInfo;

private:
    
    int mCmdNum;
    
};

class CallBack 
{

public:

    struct Queue {
        std::deque<std::shared_ptr<ITask>> q;
        std::mutex mut;
        std::condition_variable cond;
        std::shared_ptr<ITask> current_task;
    }mSTQueue;

    bool mIdel;

private:

    ros::Publisher mHivePub, mNaviPub;
    std::vector<CmdSet> mVeCmdSet;

    cti_msgs::NaviRequest mMsg;
    bool mInit;

public:
    CallBack(ros::NodeHandle node, std::string yamlfile)
    {
        mIdel = false;
        mInit = true;
        loadCmdSetFromYamlFile(yamlfile);
    }

    void loadCmdSetFromYamlFile(std::string yamlfile)
    {
        std::ifstream fin(yamlfile);
        if (fin.fail()) {
            ROS_ERROR("virtual cloud can't open %s.", yamlfile.c_str());
            exit(-1);
        }

        YAML::Node node = YAML::Load(fin);
        YAML::Node cmdset = node["cmdset"];
        YAML::Node cmdinfo = node["cmdinfo"];

        for (int i=0; i < cmdset.size(); i++)
        {
            std::string cmdsetname;
            cmdset[i] >> cmdsetname;

            CmdSet cs(cmdsetname, cmdinfo[cmdsetname.c_str()]);
            mVeCmdSet.push_back(cs);
        }

        //load relocation pose
        mMsg.action = cti_msgs::NaviRequest::ACTION_RE_LOCATION;
        node["relocation_pose"]["building"] >> mMsg.building;
        node["relocation_pose"]["level"] >> mMsg.level;
        node["relocation_pose"]["x"] >> mMsg.coordinates.position.x;
        node["relocation_pose"]["y"] >> mMsg.coordinates.position.y;
        node["relocation_pose"]["z"] >> mMsg.coordinates.orientation.z;
        node["relocation_pose"]["w"] >> mMsg.coordinates.orientation.w;
        mMsg.coordinates.position.z = mMsg.coordinates.orientation.x = mMsg.coordinates.orientation.y = 0.0;
    }

    void callbackCMD(std_msgs::String msg)
    {
        std::string cmdsetname = msg.data; 
        ROS_ERROR("msg : %s", cmdsetname.c_str());

        auto it = std::find_if(mVeCmdSet.begin(), mVeCmdSet.end(), [cmdsetname](const CmdSet& cs){
            return cs.mCmdSetName == cmdsetname;
        });

        if (it == mVeCmdSet.end())
        {
            ROS_ERROR("no such %s cmd set", cmdsetname.c_str());

            if (!cmdsetname.compare("ca"))// clean all
            {
                std::unique_lock<std::mutex> ul(mSTQueue.mut);
                std::deque<std::shared_ptr<ITask>>().swap(mSTQueue.q); //清空队列中的任务列表
            }
            else if (!cmdsetname.compare("cs"))// clean and stop
            {
                std::unique_lock<std::mutex> ul(mSTQueue.mut);
                std::deque<std::shared_ptr<ITask>>().swap(mSTQueue.q); //清空队列中的任务列表
            }
            else if (!cmdsetname.compare("size")) // get tasks num
            {
                std::unique_lock<std::mutex> ul(mSTQueue.mut);
                ROS_ERROR("QUEUE SIZE : %ld", mSTQueue.q.size());
            }
            else if (!cmdsetname.compare("re"))// stop current task and restart current task
            {
                std::unique_lock<std::mutex> ul(mSTQueue.mut);

                mSTQueue.q.push_front(mSTQueue.current_task);//把当前任务再次放到队列头
            }
            return;
            
        }

        std::unique_lock<std::mutex> ul(mSTQueue.mut);

        std::for_each(it->mVeCmdSetInfo.begin(), it->mVeCmdSetInfo.end(), 
            [=]
            (std::shared_ptr<CmdBaseInfo> cbi){
                switch (cbi->type)
                {
                    case 0:
                    {
                        ROS_ERROR("-push NaviTask--");
                        auto pnt = std::make_shared<NaviTask>();
                        pnt->initTask(mNaviPub, cbi);
                        mSTQueue.q.push_back(pnt);
                        break;
                    }
                    case 1:
                    {
                        ROS_ERROR("-push HiveTask--");
                        auto pht = std::make_shared<HiveTask>();
                        pht->initTask(mHivePub, cbi);
                        mSTQueue.q.push_back(pht);
                        break;
                    }
                    case 3:
                    {
                        ROS_ERROR("-push SleepTask--");
                        auto slt = std::make_shared<SleepTask>();
                        slt->initTask(mHivePub, cbi);
                        mSTQueue.q.push_back(slt);
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
            }
        );

        mSTQueue.cond.notify_one();
    }

    void callbackBRS(cti_msgs::BuildingRobotState msg)
    {
        if ((ros::Time::now() - msg.timestamp).toSec() > 5.0)
        {
            SPDLOG_INFO("building robot state is too old");
            return;
        }

        switch (msg.state)
        {
            case cti_msgs::BuildingRobotState::STATE_WAITTING_TASK:
            {
                ROS_ERROR("- idel --");
                mIdel = true;
                break;
            }
            case cti_msgs::BuildingRobotState::STATE_START:
            {
                ROS_ERROR("- just start --");
                if (mInit)
                {
                    mNaviPub.publish(mMsg);
                    mInit = false;
                }
                mIdel = false;
                break;
            }
            default:
            {
                ROS_ERROR("- no idel --");
                mIdel = false;
                break;
            }
        }
    } 
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "virtual_cload_node");

    auto node = ros::NodeHandle();
    auto private_node = ros::NodeHandle("~");

    std::string cmdfile;
    if (!private_node.getParam("cmd_file", cmdfile)) cmdfile = "/opt/cti/kinect/share/virtual_robot_base/param/cmdinfo.yaml";

    CallBack cb(node, cmdfile);

    auto subCmd = node.subscribe("/virtual_cloud_cmd", 1, &CallBack::callbackCMD, &cb);

    auto subBRS = node.subscribe("/robot_state", 1, &CallBack::callbackBRS, &cb);

    ROS_ERROR("-sleep 30s-");
    ros::Duration(30).sleep();

    std::thread([&cb]()
    {
        ROS_ERROR("-start thread-");
        while (true)
        {
            ROS_ERROR("-wait task--");
            //等待任务入队
            std::unique_lock<std::mutex> ul(cb.mSTQueue.mut);
            cb.mSTQueue.cond.wait(ul, [&cb](){return !cb.mSTQueue.q.empty();});
            ROS_ERROR("-pop Task and do it--");
            //发布任务
            (cb.mSTQueue.q.front())->pubTask();
            cb.mSTQueue.current_task = cb.mSTQueue.q.front();
            cb.mSTQueue.q.pop_front();
            ul.unlock();

            do {
                ROS_ERROR("-wait idel 2s--");
                std::this_thread::sleep_for(2s);
            }while (!cb.mIdel);
        }
    }).detach();

    ros::spin();

    return 0;
}
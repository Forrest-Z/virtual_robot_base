
#include <ros/ros.h>
#include <utility>
#include <thread>
#include <algorithm>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <cti_msgs/BuildingRobotState.h>
#include "candle_common/proto/local_schedule_message.pb.h"

#include <visualization_msgs/MarkerArray.h>
using namespace std::literals;

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

std::string baseFrameName, scanFrameName, scanTopicName, mapTopicName, mapFrameName, gRecMarkerTopic;

int sfold(std::string s)
{
    int M = 60;
    int intLength = s.size() / 4;
    long sum = 0;
    for (int j = 0; j < intLength; j++)
    {
        std::string sub_string = s.substr(j * 4, (j * 4) + 4);
        char c[sub_string.size()];
        std::strcpy(c, sub_string.c_str());
        long mult = 1;
        for (int k = 0; k < strlen(c); k++)
        {
        sum += c[k] * mult;
        mult *= 256;
        }
    }

    std::string sub_string = s.substr(intLength * 4);
    char c[sub_string.size()];
    std::strcpy(c, sub_string.c_str());
    long mult = 1;
    for (int k = 0; k < strlen(c); k++)
    {
        sum += c[k] * mult;
        mult *= 256;
    }

    return(std::abs(sum) % M);
}

geometry_msgs::Pose convert_index_to_position(int x_index,
                                              int y_index,
                                              double resolution)
{
    geometry_msgs::Pose return_value;
    return_value.position.x = resolution * x_index;
    return_value.position.y = resolution * y_index;
    return_value.orientation.x = 0.0;
    return_value.orientation.y = 0.0;
    return_value.orientation.z = 0.0;
    return_value.orientation.w = 1.0;
    return return_value;
}

struct map_info_s
{
    nav_msgs::OccupancyGrid mMapGrid;
    nav_msgs::OccupancyGrid mDynamicMapGrid;
    std::mutex mMapMutrx;

    std::mutex mRobotInfoMutrx;

    int mBuildingNameHash;
    std::string mFloorName;
    std::string mRobotNum;

    typedef struct RobotInfo
    {
        std::string robot_id;
        int buildingname_hash;
        std::string floor_name;
        geometry_msgs::Pose current_pose;
        ros::Time update_time;
        visualization_msgs::Marker maker;
    }stRobotInfo;

    std::vector<stRobotInfo> mRobotsInfo;
    std::map<int,visualization_msgs::Marker> mObstacleMaps;

    struct PointInt
    {
        int x;
        int y;
    };
    struct PointDouble
    {
        double x;
        double y;
    };

    void raytrace(int x0, int y0, int x1, int y1, std::vector<PointInt> &cells)
    {
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        PointInt pt;
        pt.x = x0;
        pt.y = y0;
        int n = 1 + dx + dy;
        int x_inc = (x1 > x0) ? 1 : -1;
        int y_inc = (y1 > y0) ? 1 : -1;
        int error = dx - dy;
        dx *= 2;
        dy *= 2;

        for (; n > 0; --n) {
            cells.push_back(pt);

            if (error > 0) {
            pt.x += x_inc;
            error -= dy;
            } else {
            pt.y += y_inc;
            error += dx;
            }
        }
    }

    void map_server_handler(const nav_msgs::OccupancyGrid& value)
    {
        std::unique_lock<std::mutex> lk_robot(mRobotInfoMutrx);
        mRobotsInfo.clear();
        lk_robot.unlock();

        std::unique_lock<std::mutex> lk_map(mMapMutrx);
        mMapGrid = value;
        mDynamicMapGrid = value;
        lk_map.unlock();
    }

    void location2Point(double __x, double __y, int& x, int& y)
    {
        if (mDynamicMapGrid.info.resolution != 0)
        {
            __x -= mDynamicMapGrid.info.origin.position.x;
            __y -= mDynamicMapGrid.info.origin.position.y;
            x = static_cast<int>(__x / mDynamicMapGrid.info.resolution);
            y = static_cast<int>(__y / mDynamicMapGrid.info.resolution);
        }
        else
        {
            x = y = -1;
        }
    }

    double detectionMap(double __x, double __y, double yaw, double min, double max)
    {
        std::unique_lock<std::mutex> lk(mMapMutrx);
        
        double l, depth = -1;
        for(l=0;l<max;l+=mDynamicMapGrid.info.resolution)
        {
            int mx, my, index = 0;
            double lx = __x + l * ::cos(yaw);
            double ly = __y + l * ::sin(yaw);
            location2Point(lx, ly, mx, my);
            if (mx >= 0 && my >= 0 && ((index = mx + my * mDynamicMapGrid.info.width) < mDynamicMapGrid.data.size()))
            {
                auto data = mDynamicMapGrid.data[index];
                if (data > 0)
                {
                    depth = ::sqrtl(::pow(lx - __x, 2) + ::pow(ly - __y, 2));
                    if (depth > 0.02) depth -= 0.02;
                    break;
                }
            }
            else
            {
                break;
            }
        }
        if (l>=max)
        {
            depth = max - 0.1;
        }
        return depth;
    }

    void robotState(const cti_msgs::BuildingRobotState::ConstPtr & msg)
    {
        std::unique_lock<std::mutex> lk(mRobotInfoMutrx);
        if (mBuildingNameHash != sfold(msg->buildingname) || mFloorName != msg->current_floor)
        {
            mBuildingNameHash = sfold(msg->buildingname);
            mFloorName = msg->current_floor;
        }
        return;
    }

    void ZigbeeCallBack(const std_msgs::String::ConstPtr & msg)
    {
        std::string input_string = msg->data;
        candle::common::AgentMessage agent_info;
        agent_info.ParseFromString(input_string);

        if (!agent_info.has_agent_id() || agent_info.agent_id() < 0)
        {
            //无机器id,id异常
            return;
        }

        std::string robot_id = std::to_string(agent_info.agent_id());
        if (robot_id == mRobotNum)
        {
            //本机的信息
            return;
        }

        std::string current_floor_number = std::to_string(-100);
        int building_hash_value = -1;

        if (agent_info.current_floor_size() > 1)
        {
            current_floor_number
                = std::to_string(agent_info.current_floor(0));
            building_hash_value = agent_info.current_floor(1);
        }

        if (mBuildingNameHash != building_hash_value || mFloorName != current_floor_number)
        {
            //不在同一楼或同一层
            return;
        }

        geometry_msgs::Pose temp_pose;
        if (agent_info.agent_position_size() > 1)
        {
            int temp_x_index = agent_info.agent_position(0);
            int temp_y_index = agent_info.agent_position(1);

            temp_pose = convert_index_to_position(temp_x_index, temp_y_index, 0.01f);
        }
        else
        {
            //机器坐标异常
            return;
        }

        std::unique_lock<std::mutex> lk(mRobotInfoMutrx);

        auto it = std::find_if(mRobotsInfo.begin(), mRobotsInfo.end(), [robot_id](stRobotInfo robotinfo) -> bool { 
            if (robotinfo.robot_id == robot_id) return true;
            return false;
        });

        if (it == mRobotsInfo.end())
        {
            stRobotInfo new_robot_info;
            new_robot_info.robot_id = robot_id;
            new_robot_info.buildingname_hash = building_hash_value;
            new_robot_info.floor_name = current_floor_number;
            new_robot_info.update_time = ros::Time::now();
            new_robot_info.current_pose = temp_pose;
            new_robot_info.maker.type = visualization_msgs::Marker::CUBE;
            new_robot_info.maker.pose = temp_pose;
            new_robot_info.maker.scale.x = 0.85;
            new_robot_info.maker.scale.y = 0.52;
            mRobotsInfo.push_back(new_robot_info);
        }
        else
        {
            it->current_pose = temp_pose;
            it->update_time = ros::Time::now();
        }

        return;
    }
    void ObstacleCallBack(const visualization_msgs::MarkerArray::ConstPtr & vObs)
    {
        
        // ROS_ERROR("vObs size %d",vObs->markers.size());
        
        for(auto obs:vObs->markers)
        {
            mObstacleMaps[obs.id] = obs;
        }
       
    }

    void setRunberNum(std::string num)
    {
        mRobotNum = num;
        return;
    }

    void updateMarkersIntoMap()
    {
        std::unique_lock<std::mutex> lk_1(mMapMutrx);
        std::unique_lock<std::mutex> lk_2(mRobotInfoMutrx);   

        mDynamicMapGrid = mMapGrid;

        std::for_each(mRobotsInfo.begin(), mRobotsInfo.end(),
            [this](stRobotInfo it){

                if ((ros::Time::now()-it.update_time).toSec() > 10)
                {
                    return;
                }

                switch (it.maker.type)
                {
                    //球形和圆柱投影到2D地图上默认是圆形
                    case visualization_msgs::Marker::SPHERE :
                    case visualization_msgs::Marker::CYLINDER :
                    {
                        double ox = it.maker.pose.position.x;
                        double oy = it.maker.pose.position.y;
                        double radius = it.maker.scale.x / 2;
                        
                        double step = M_PI*2/720; // 720 points
                        for (double angle = 0.0; angle < M_PI*2; angle+=step)
                        {
                            double dx = ox + radius * ::cos(angle);
                            double dy = oy + radius * ::sin(angle);

                            int mx, my, index;
                            this->location2Point(dx, dy, mx, my);
                            if (mx >= 0 && my >= 0 && ((index = mx + my * mDynamicMapGrid.info.width) < mDynamicMapGrid.data.size()))
                            {
                                mDynamicMapGrid.data[index] = 100;
                            }
                        }
                        break;
                    }
                    //立方体投影到2D地图上默认是长方形
                    case visualization_msgs::Marker::CUBE :
                    {
                        // double ox = it.maker.pose.position.x;
                        // double oy = it.maker.pose.position.y;
                        double ox = it.current_pose.position.x;
                        double oy = it.current_pose.position.y;
                        double oAngle = tf::getYaw(it.maker.pose.orientation);

                        double lx = it.maker.scale.x / 2;
                        double ly = it.maker.scale.y / 2;
                        double radius = std::sqrt(::pow(lx, 2) + ::pow(ly, 2));

                        double lAngle[4];
                        lAngle[0] = ::atan(ly/lx);
                        lAngle[1] = M_PI - ::atan(ly/lx);
                        lAngle[2] = M_PI - ::atan(0-ly/lx);
                        lAngle[3] = ::atan(0-ly/lx);

                        PointInt point[4];
                        std::vector<PointInt> cells;

                        for (int i=0; i<4; i++){                            
                            this->location2Point(ox + radius * ::cos(oAngle + lAngle[i]), 
                                                oy + radius * ::sin(oAngle + lAngle[i]), 
                                                point[i].x, 
                                                point[i].y);
                        }

                        raytrace(point[0].x, point[0].y, point[1].x, point[1].y, cells);
                        raytrace(point[1].x, point[1].y, point[2].x, point[2].y, cells);
                        raytrace(point[2].x, point[2].y, point[3].x, point[3].y, cells);
                        raytrace(point[3].x, point[3].y, point[0].x, point[0].y, cells);

                        for (int k = 0; k < cells.size(); k++)
                        {
                            mDynamicMapGrid.data[MAP_IDX(mDynamicMapGrid.info.width, cells[k].x , cells[k].y)] = 100;
                        }

                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
                return;
            }
        );
    }
    void updateObstaclesIntoMap()
    {
        std::unique_lock<std::mutex> lk_1(mMapMutrx);
    	// mDynamicMapGrid = mMapGrid;
        for(auto obs:mObstacleMaps)
        {
            visualization_msgs::Marker marker = obs.second;
            double ox = marker.pose.position.x;
            double oy = marker.pose.position.y;
            double oAngle = tf::getYaw(marker.pose.orientation);
            double cs = cos(oAngle);
            double sn = sin(oAngle);
            double hx = marker.scale.x / 2;
            double hy = marker.scale.y / 2;

            // ROS_ERROR(":id %d (%f %f)",marker.id,ox,oy);
            PointDouble bodyRect[4];
            bodyRect[0].x = -hx;bodyRect[0].y = -hy;
            bodyRect[1].x =  hx;bodyRect[1].y = -hy;
            bodyRect[2].x =  hx;bodyRect[2].y =  hy;
            bodyRect[3].x = -hx;bodyRect[3].y =  hy;
         
            PointInt point[4];
            PointDouble mapRect[4];
            std::vector<PointInt> cells;
            
            for (int i=0; i<4; i++)
            {                  
                mapRect[i].x = cs * bodyRect[i].x + sn * bodyRect[i].y + ox;
                mapRect[i].y = -1.0*sn * bodyRect[i].x + cs * bodyRect[i].y + oy;
                this->location2Point(mapRect[i].x,mapRect[i].y,point[i].x,point[i].y);  
            }
            
            raytrace(point[0].x, point[0].y, point[1].x, point[1].y, cells);
            raytrace(point[1].x, point[1].y, point[2].x, point[2].y, cells);
            raytrace(point[2].x, point[2].y, point[3].x, point[3].y, cells);
            raytrace(point[3].x, point[3].y, point[0].x, point[0].y, cells);

            for (int k = 0; k < cells.size(); k++)
            {
                mDynamicMapGrid.data[MAP_IDX(mDynamicMapGrid.info.width, cells[k].x , cells[k].y)] = 100;
            }  
        }
    }
} map_info;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "virtual_laser_sensor_node");

    std::thread([node = std::make_shared<ros::NodeHandle>("~")]
    {
        if (!node->getParam("base_frame", baseFrameName)) baseFrameName = "base_link";
        if (!node->getParam("map_topic", mapTopicName)) mapTopicName = "/navigation_map";
        if (!node->getParam("map_frame", mapFrameName)) mapTopicName = "map";
        if (!node->getParam("lanser_topic", scanTopicName)) scanTopicName = "/scan";
        if (!node->getParam("laser_frame", scanFrameName)) scanFrameName = "laser_merge";
        if (!node->getParam("marker", gRecMarkerTopic)) gRecMarkerTopic = "/marker";

        const int num_of_point = 1440;
        auto scanPublisher = node->advertise<sensor_msgs::LaserScan>(scanTopicName, 32);
        auto frontPublisher = node->advertise<sensor_msgs::LaserScan>("/front_scan", 32);
        auto rearPublisher = node->advertise<sensor_msgs::LaserScan>("/rear_scan", 32);
        auto frontScanPublisher = node->advertise<sensor_msgs::LaserScan>("/front_scan_filter", 32);
        auto mapSubscriber = node->subscribe(mapTopicName, 2, &map_info_s::map_server_handler, &map_info);
        // auto markerSubscriber = node->subscribe(gRecMarkerTopic, 1, &map_info_s::markerCallback, &map_info);
        auto mapPublisher = node->advertise<nav_msgs::OccupancyGrid>("/obstacle_map", 1);

        tf::TransformListener tfListener;
        int count = 0;
        while(ros::ok())
        {
            double x, y, yaw;
            tf::StampedTransform transform;
            try
            {
                tfListener.lookupTransform(mapFrameName, scanFrameName, ros::Time(0), transform);
                x = transform.getOrigin().x();
                y = transform.getOrigin().y();
                yaw = tf::getYaw(transform.getRotation());
                if (count++ % 50 == 0)
                {
                    std::cout << "x:" << x << ", y:" << y << ", yaw:" << yaw << std::endl;
                }
            }
            catch(std::exception&ex)
            {
                try
                {
                    tfListener.waitForTransform(mapFrameName, scanFrameName, ros::Time(0), ros::Duration(2));
                }
                catch(std::exception&)
                {
        
                }
                std::cerr << ex.what() << std::endl;
                std::this_thread::sleep_for(1s);
            }
            
            sensor_msgs::LaserScan data;
            data.header.frame_id = scanFrameName;
            data.header.stamp = ros::Time::now();
            data.angle_min = -M_PI;
            data.angle_max = M_PI;
            data.angle_increment = 2 * M_PI / num_of_point;
            data.range_max = 10;
            data.range_min = 0.1;
            data.time_increment = 0.05 / num_of_point;

            data.ranges.resize(num_of_point);
            data.intensities.resize(num_of_point);

            std::fill(data.ranges.begin(), data.ranges.end(), -1);

            for (int i=0;i<data.ranges.size();i++)
            {
                data.ranges[i] = map_info.detectionMap(x, y, yaw + i * data.angle_increment + data.angle_min, data.range_min, data.range_max);
            }

            scanPublisher.publish(data);
            frontPublisher.publish(data);
            rearPublisher.publish(data);
            frontScanPublisher.publish(data);
            mapPublisher.publish(map_info.mDynamicMapGrid);

            std::this_thread::sleep_for(50ms);
        }
    }).detach();



    std::thread([node = std::make_shared<ros::NodeHandle>("~")]
    {
        std::string robot_num;
	    node->getParam("/robot_attribute/number", robot_num);
        map_info.setRunberNum(robot_num);

        auto zigbeeState = node->subscribe("/local_communication_received_message", 100, &map_info_s::ZigbeeCallBack, &map_info);
        auto dynamicobs = node->subscribe("/dynamic_obstacle_marker", 1, &map_info_s::ObstacleCallBack, &map_info);
        auto dragobs = node->subscribe("/drag_obstacle_marker", 1, &map_info_s::ObstacleCallBack, &map_info);
        auto robot_state = node->subscribe("/robot_state", 1, &map_info_s::robotState, &map_info);
        while(ros::ok())
        {
            std::this_thread::sleep_for(50ms);
            map_info.updateMarkersIntoMap();
            map_info.updateObstaclesIntoMap();
        }
    }).detach();

    ros::spin();
}
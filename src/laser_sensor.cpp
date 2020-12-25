/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @Date: 2019-08-30 15:19:06
 * @LastEditTime: 2020-04-21 09:09:03
 * @LastEditors: Please set LastEditors
 */
 /**  Created on: 2019年5月06日
  *      Author: liuquan
  * 
  * 伪造激光传感器数据
  * 先获取当前机器人在地图中的位置，依照周围地图障碍情况，构造虚拟激光雷达数据并发布
  */

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

using namespace std::literals;

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

std::string baseFrameName, scanFrameName, scanTopicName, mapTopicName, mapFrameName, gRecMarkerTopic;

struct map_info_s
{
    nav_msgs::OccupancyGrid mMapGrid;
    nav_msgs::OccupancyGrid mDynamicMapGrid;
    std::vector<visualization_msgs::Marker> mVecMarkers;

    std::mutex mMapMutrx;

    struct PointInt
    {
        int x;
        int y;
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
        std::lock_guard<std::mutex> lk(mMapMutrx);

        mMapGrid = value;
        mDynamicMapGrid = value;
        mVecMarkers.clear();
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
        std::lock_guard<std::mutex> lk(mMapMutrx);
        
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

    void markerCallback(const visualization_msgs::Marker& marker)
    {

        std::lock_guard<std::mutex> lk(mMapMutrx);

        switch(marker.action)
        {
            case visualization_msgs::Marker::ADD :  // ADD or MODIFY
            // case visualization_msgs::Marker::MODIFY :
            {
                if (std::find_if(mVecMarkers.begin(), mVecMarkers.end(), 
                    [marker](visualization_msgs::Marker it){
                            return marker.id == it.id;}) 
                    != mVecMarkers.end())
                {
                    mVecMarkers.erase(
                        std::remove_if(mVecMarkers.begin(), mVecMarkers.end(), 
                            [marker](visualization_msgs::Marker it){
                                return marker.id == it.id;})
                    );
                }
                
                mVecMarkers.push_back(marker);
                break;
            }
            case visualization_msgs::Marker::DELETE :
            {
                if (std::find_if(mVecMarkers.begin(), mVecMarkers.end(), 
                    [marker](visualization_msgs::Marker it){
                            return marker.id == it.id;}) 
                    != mVecMarkers.end())
                {
                    mVecMarkers.erase(
                        std::remove_if(mVecMarkers.begin(), mVecMarkers.end(), 
                            [marker](visualization_msgs::Marker it){
                                return marker.id == it.id;})
                    );
                }

                break;
            }
            case visualization_msgs::Marker::DELETEALL :
            {
                mVecMarkers.clear();
                break;
            }
            default:
            {
                break;
            }
        }

        ROS_ERROR("mVecMarkers size %ld", mVecMarkers.size());

        mDynamicMapGrid = mMapGrid;

        std::for_each(mVecMarkers.begin(), mVecMarkers.end(),
            [this](visualization_msgs::Marker it){
                switch (it.type)
                {
                    //球形和圆柱投影到2D地图上默认是圆形
                    case visualization_msgs::Marker::SPHERE :
                    case visualization_msgs::Marker::CYLINDER :
                    {
                        double ox = it.pose.position.x;
                        double oy = it.pose.position.y;
                        double radius = it.scale.x / 2;
                        
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
                        double ox = it.pose.position.x;
                        double oy = it.pose.position.y;
                        double oAngle = tf::getYaw(it.pose.orientation);

                        double lx = it.scale.x / 2;
                        double ly = it.scale.y / 2;
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
        auto markerSubscriber = node->subscribe(gRecMarkerTopic, 1, &map_info_s::markerCallback, &map_info);

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

            std::this_thread::sleep_for(50ms);
        }
    }).detach();

    ros::spin();
}
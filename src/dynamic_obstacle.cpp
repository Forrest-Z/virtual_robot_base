#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include "dynamic_obstacle.hpp"
#include "drag_obstacle.hpp"
#include <visualization_msgs/MarkerArray.h>

bool loadObstacles(std::vector<SimRobot> &vRobots)
{
    ros::NodeHandle private_nh("~/");

    vRobots.clear();

    if (private_nh.hasParam("move_straight_obstacle"))
    {

        XmlRpc::XmlRpcValue xml_obstacle_params;
        private_nh.getParam("move_straight_obstacle", xml_obstacle_params);

        for (size_t i = 0; i < xml_obstacle_params.size(); ++i)
        {
            double speed = xml_obstacle_params[i]["speed"];
            int obs_id = xml_obstacle_params[i]["id"];
            ROS_ERROR("speed %f", speed);
            XmlRpc::XmlRpcValue start_point = xml_obstacle_params[i]["start"];
            geometry_msgs::Point pStart;
            pStart.x = (double)(start_point[0]);
            pStart.y = (double)(start_point[1]);

            XmlRpc::XmlRpcValue end_point = xml_obstacle_params[i]["end"];
            geometry_msgs::Point pGoal;
            pGoal.x = (double)(end_point[0]);
            pGoal.y = (double)(end_point[1]);
            ROS_ERROR("[%d] (%f %f) (%f %f)", i, pStart.x, pStart.y, pGoal.x, pGoal.y);

            SimRobot r(obs_id,speed,pStart,pGoal);
            vRobots.push_back(r);
        }
        return true;
    }
    else
    {
        ROS_ERROR("full name: %s", private_nh.getUnresolvedNamespace().c_str());
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamic_obstacle");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/dynamic_obstacle_marker", 0);
    ros::Publisher drag_pub = nh.advertise<visualization_msgs::MarkerArray>("/drag_obstacle_marker", 0);
    
   
   
    std::vector<SimRobot> vObstacle;
    if (!loadObstacles(vObstacle))
    {
        ros::shutdown();
        return 0;
    }

    ROS_ERROR("size : %d",vObstacle.size());
    DragObstacle DO;
    while (ros::ok())
    {
        visualization_msgs::MarkerArray marker_array;
  
        for(auto &obs : vObstacle)
        {
            obs.update();
            marker_array.markers.push_back(obs.makeMarker());
        }

        std::map<std::string, visualization_msgs::Marker> mMakers = DO.GetMarkers();
        visualization_msgs::MarkerArray drag_marker_array;
        for(auto &m : mMakers)
        {
            drag_marker_array.markers.push_back(m.second);
            // ROS_ERROR("-----name: %s pose (%f %f %f)",m.first.c_str(),m.second.pose.position.x,m.second.pose.position.y,m.second.pose.position.z);
        }
        vis_pub.publish( marker_array );
        drag_pub.publish( drag_marker_array );
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
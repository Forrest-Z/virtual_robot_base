#ifndef DYOBSTACLE_HPP_
#define DYOBSTACLE_HPP_
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <algorithm>
#include <tf/tf.h>
// #include <Eigen/Core>
#include <visualization_msgs/Marker.h>

using namespace geometry_msgs;
class SimRobot
{
public:
    SimRobot(int id, double sp, Point s, Point g) : sim_id_(id), speed_(sp), start_(s), goal_(g)
    {
        sim_id_ += 1000;
        client = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
        orientation_.w = 1.0;
        current_ = start_;
        make_plan_success = make_plan(start_,goal_);
        last_make_plan = ros::Time::now();
    }

    double getDistance(Point p1, Point p2)
    {
        return std::hypot(p1.x - p2.x, p1.y - p2.y);
    }

    Point getPosition()
    {
        return current_;
    }

    bool isReachGoal(void)
    {
        return (getDistance(current_,goal_) < 0.05);
    }

    bool make_plan(geometry_msgs::Point &s,geometry_msgs::Point &g)
    {
        nav_msgs::GetPlan srv;
        geometry_msgs::PoseStamped start;
        geometry_msgs::PoseStamped goal;
        

        start.header.frame_id = "map";
        start.pose.position = s;

        goal.header.frame_id = "map";
        goal.pose.position = g;

        srv.request.start = start;
        srv.request.goal = goal;
        if (client.call(srv))
        {
            ROS_ERROR("make plan OK");
            for(auto p:srv.response.plan.poses)
            plan_.push_back(p.pose.position);
            return true;
        }
        else
        {
            ROS_ERROR("Failed make plan");
            return false;
        }
    }

    void update(void)
    {
        // if(!make_plan_success)
        // {
        //     make_plan_success = make_plan(start_,goal_);
        //     return;
        // }
        if (!start_move)
        {
            last_move_time = ros::Time::now();
            start_move = true;
            prvPoint_.x = 0.0;
            prvPoint_.y = 0.0;
            return;
        }
        
        double dt = (ros::Time::now() - last_move_time).toSec();
        last_move_time = ros::Time::now();
        double move_dist = dt * speed_;

        // if((ros::Time::now()- last_make_plan).toSec() > 1)
        // {
        //     make_plan_success = make_plan(start_,goal_);
        //     last_make_plan = ros::Time::now();
        // }
        int plan_size = plan_.size();



        if(plan_.empty())
        {
            return ;
        }
        if(isReachGoal()|| vector_indx >  (plan_size - 1))
        {
            current_ = plan_.back();
            reverse(plan_.begin(),plan_.end());
            start_ = plan_.front();
            goal_ = plan_.back();
            vector_indx = 0;
        }

        for(;vector_indx < plan_size;vector_indx++)
        {
            Point p = plan_[vector_indx];
            if(getDistance(current_,p) > move_dist)
            {
                current_ = p;
                break;
            }
        }
  
        double yaw = atan2((current_.y-prvPoint_.y),(current_.x-prvPoint_.x));
        prvPoint_ = current_;
        orientation_ = tf::createQuaternionMsgFromYaw(yaw);
       
    }

    visualization_msgs::Marker makeMarker(void)
    {
        visualization_msgs::Marker obsMarker;
        obsMarker.header.frame_id = "/map";
        obsMarker.header.stamp = ros::Time::now();
        obsMarker.ns = "Obstacle";
        obsMarker.action = visualization_msgs::Marker::ADD; // visualization_msgs::Marker::DELETE
        obsMarker.id = sim_id_;
        obsMarker.type = visualization_msgs::Marker::CUBE;
        
        obsMarker.scale.x = 0.85;
        obsMarker.scale.y = 0.52;
        obsMarker.scale.z = 0.5;
        obsMarker.color.r = 0.5;
        obsMarker.color.b = 1.0;
        obsMarker.color.a = 1.0;
        obsMarker.pose.position = current_;
        obsMarker.pose.orientation = orientation_;

        return obsMarker;
    }

private:
    ros::NodeHandle nh;
    ros::ServiceClient client;
    double speed_;

    int sim_id_;

    bool start_move{false};
    std::vector<Point> plan_;

    double distance;
    bool make_plan_success{false};
    ros::Time last_move_time;
    ros::Time last_make_plan;
    int vector_indx{0};
    geometry_msgs::Point start_;
    geometry_msgs::Point goal_;
    geometry_msgs::Point current_;
    geometry_msgs::Point prvPoint_;
    geometry_msgs::Quaternion orientation_;
};

#endif
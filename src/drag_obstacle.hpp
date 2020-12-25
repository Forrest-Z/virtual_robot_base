
#ifndef DRAGOBSTACLE_H_PP_
#define DRAGOBSTACLE_H_PP_
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>

using namespace geometry_msgs;
using namespace visualization_msgs;
class DragObstacle
{
public:
    ~DragObstacle() {}
    DragObstacle()
    {
        server.reset(new interactive_markers::InteractiveMarkerServer("obstcals", "", false));
        geometry_msgs::Pose pose;
        pose.position.x = -1.0;
        pose.position.z = 0.25;
        pose.orientation.w = 1.0;
        makeChessPieceMarker("A", pose);
        pose.position.y = 1;
        makeChessPieceMarker("B", pose);
        pose.position.x = 1.0;
        makeChessPieceMarker("C", pose);
    }



    visualization_msgs::Marker makeBox(void)
    {
        static int static_marker_id = 2000;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "Obstacle";
        marker.id = static_marker_id++;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        return marker;
    }

    void makeChessPieceMarker(std::string name, const geometry_msgs::Pose pose)
    {
        InteractiveMarker int_marker;
        int_marker.header.frame_id = "/map";

        int_marker.scale = 10;
        InteractiveMarkerControl control;
        tf::Quaternion orien(0.0, 1.0, 0.0, 1.0);
        orien.normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
        visualization_msgs::Marker m = makeBox();
        m.pose = pose;
        control.markers.push_back(m);
        mMarkers[name] = m;
        int_marker.controls.push_back(control);

        int_marker.pose = pose;
        int_marker.name = name;

        control.always_visible = true;
        server->insert(int_marker, boost::bind(&DragObstacle::alignMarker, this, _1));
        server->applyChanges();
    }
    void alignMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        server->setPose(feedback->marker_name, feedback->pose);
        mMarkers[feedback->marker_name].pose = feedback->pose;
        // ROS_ERROR("name: %s pose (%f %f %f)",feedback->marker_name.c_str(),feedback->pose.position.x,feedback->pose.position.y,feedback->pose.position.z);
        server->applyChanges();
    }

    std::map<std::string, visualization_msgs::Marker> GetMarkers(void)
    {
        return mMarkers;
    }
private:
    // ros::NodeHandle nh;
    std::map<std::string, visualization_msgs::Marker> mMarkers;
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

};
#endif
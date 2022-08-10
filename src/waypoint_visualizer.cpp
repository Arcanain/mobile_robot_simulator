#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>

class Waypoint_Visualizer
{
    private:
        ros::NodeHandle nh;
        // publisher
        ros::Publisher marker_pub;
        ros::Publisher markerText_pub;
        // subscriber
        ros::Subscriber path_sub;
        ros::Subscriber targetwp_num_sub;   
        //callback variable
        nav_msgs::Path path;
        //std_msgs::Int32 targetwp_num;
        int targetwp_num;
        //marker variable
        double markerSize = 1.0;
        double marker_diameter = 0.1 * markerSize;
        double marker_height = 0.03 * markerSize;
        double text_size = 0.1 * markerSize;
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::MarkerArray markerText_array;
        visualization_msgs::MarkerArray marker_empty_array;
        visualization_msgs::MarkerArray markerText_empty_array;
    public:
        Waypoint_Visualizer();
        ~Waypoint_Visualizer();
        void path_callback(const nav_msgs::Path &path_msg);
        void targetwp_num_callback(const std_msgs::Int32 &targetwp_num_msg);
        void marker_publish();
        // color set
        std_msgs::ColorRGBA green;
        std_msgs::ColorRGBA red;
        std_msgs::ColorRGBA blue;
        std_msgs::ColorRGBA gray;
        std_msgs::ColorRGBA black;
        std_msgs::ColorRGBA set_color(double r, double g, double b, double a);
};

Waypoint_Visualizer::Waypoint_Visualizer()
{
    //publisher
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/waypoint/marker", 10);
    markerText_pub = nh.advertise<visualization_msgs::MarkerArray>("/waypoint/markerText", 10);
    //subscriber
    path_sub = nh.subscribe("/path", 10, &Waypoint_Visualizer::path_callback, this);
    targetwp_num_sub = nh.subscribe("/targetwp_num", 10, &Waypoint_Visualizer::targetwp_num_callback, this);

    //color set
    green = set_color(0.0, 1.0, 0.0, 1.0);
    red   = set_color(1.0, 0.0, 0.0, 1.0);
    blue  = set_color(0.0, 0.0, 1.0, 1.0);
    gray  = set_color(0.3, 0.3, 0.3, 1.0);
    black = set_color(0.0, 0.0, 0.0, 1.0);
}

Waypoint_Visualizer::~Waypoint_Visualizer()
{
    std::cout << "Finish Waypoint Visualizer!" << std::endl;
}

void Waypoint_Visualizer::path_callback(const nav_msgs::Path &path_msg)
{
    path = path_msg;
}

void Waypoint_Visualizer::targetwp_num_callback(const std_msgs::Int32 &targetwp_num_msg)
{
    targetwp_num = targetwp_num_msg.data;
    //std::cout << targetwp_num << std::endl;
}

void Waypoint_Visualizer::marker_publish()
{   
    for (int i = 0; i < path.poses.size(); i++) {
        visualization_msgs::Marker marker;
        visualization_msgs::Marker markerText;
        
        // marker set
        marker.header.frame_id = path.header.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "waypoint_marker";
        marker.id = i;
        marker.lifetime = ros::Duration();
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = marker_diameter;
        marker.scale.y = marker_diameter;
        marker.scale.z = marker_height;
        marker.pose = path.poses.at(i).pose;

        //target
        if(targetwp_num == i){
            marker.color = red;
        }
        if(targetwp_num < i){
            marker.color = blue;
        }
        //end target
        if(targetwp_num > i){
            marker.color = gray;
        }
        
        // markerText set
        markerText.header.frame_id = path.header.frame_id;
        markerText.header.stamp = ros::Time::now();
        markerText.ns = "waypoint_marker";
        markerText.id = i;
        markerText.lifetime = ros::Duration();
        markerText.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        markerText.scale.x = text_size;
        markerText.scale.y = text_size;
        markerText.scale.z = text_size;
        markerText.pose = path.poses.at(i).pose;
        markerText.pose.position.z+=marker_height;
        markerText.text= std::to_string(i).c_str();
        markerText.color = black;

        // push back array
        if (i == targetwp_num - 1 || i == targetwp_num || i == targetwp_num + 1) {
            marker_array.markers.push_back(marker);
            markerText_array.markers.push_back(markerText);
        }
    }

    marker_pub.publish(marker_array);
    markerText_pub.publish(markerText_array);
    // marker_array to empty
    marker_array = marker_empty_array;
    markerText_array = markerText_empty_array;
}

std_msgs::ColorRGBA Waypoint_Visualizer::set_color(double r, double g, double b, double a)
{
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    return color;
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "waypoint_visualizer");
    
    Waypoint_Visualizer waypoint_visualizer;
    ros::Rate loop_rate(50); // 50Hz
	while(ros::ok()){
		ros::spinOnce();
        waypoint_visualizer.marker_publish();
		loop_rate.sleep();
	}

    return 0;
}
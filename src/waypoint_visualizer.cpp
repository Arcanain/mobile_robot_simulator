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
    public:
        Waypoint_Visualizer();
        ~Waypoint_Visualizer();
        void path_callback(const nav_msgs::Path &path_msg);
        void targetwp_num_callback(const std_msgs::Int32 &targetwp_num_msg);
        void marker_publish();
        // color set
        std_msgs::ColorRGBA green;
        std_msgs::ColorRGBA red;
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
            marker.color = green;
        }
        //end target
        if(targetwp_num > i){
            marker.color = gray;
        }
        
        marker_array.markers.push_back(marker);
    }

    marker_pub.publish(marker_array);
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
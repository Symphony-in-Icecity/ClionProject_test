#include "ros/ros.h"
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */


std::vector<geometry_msgs::Point> init_cars_position()
{
    std::vector<geometry_msgs::Point> cars_position(10);
    for (int i = 0; i < cars_position.size(); i++)
    {
        cars_position[i].x = 3 * i;
        cars_position[i].y = 0;
        cars_position[i].z = 0;
    }

    return cars_position;
}

void update_car_position(std::vector<geometry_msgs::Point> &cars_position)
{
    for (int i = 0; i < cars_position.size(); i++)
    {
        cars_position[i].x = cars_position[i].x + 1;
        cars_position[i].y = cars_position[i].y + 2;
    }
}

void display_cars(std::vector<geometry_msgs::Point> &cars_position, visualization_msgs::MarkerArray &cars_marker)
{
    for (int i = 0; cars_position.size(); i++)
    {
        visualization_msgs::Marker car_marker;
        car_marker.id = i + 1;
        car_marker.header.frame_id = "map";
        car_marker.header.stamp = ros::Time();
        car_marker.ns = "car_position";
        car_marker.type = visualization_msgs::Marker::POINTS;
        car_marker.action = visualization_msgs::Marker::MODIFY;
        car_marker.scale.x = 0.1;
        car_marker.scale.y = 0.1;
        car_marker.color.a = 0.9;
        car_marker.color.r = 1.0;
        car_marker.color.g = 1.0;
        car_marker.color.b = 0.0;
        car_marker.points.push_back(cars_position[i]);
        cars_marker.markers.push_back(car_marker);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "car_display");

    ros::NodeHandle nh;

    ros::Publisher cars_display_pub = nh.advertise<visualization_msgs::MarkerArray>("cars_position_display", 1);

    std::vector<geometry_msgs::Point> cars_start_position = init_cars_position();
    std::vector<geometry_msgs::Point> cars_position = cars_start_position;

    visualization_msgs::MarkerArray cars_marker;

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        display_cars(cars_position, cars_marker);
        cars_display_pub.publish(cars_marker);
        update_car_position(cars_position);

        ros::spinOnce();

        loop_rate.sleep();
        cars_marker.markers.clear();
    }

    return 0;
}
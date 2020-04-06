#include <ros/ros.h>
#include <navigation_pkg/AStar.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

double resolution;
navigation_pkg::Vector3 origin;
int height, width;
std::vector<std::vector<int>> data;
bool checkForSpin = false;

double getYawFromQuaternion(geometry_msgs::Quaternion q){
    tf::Quaternion Q(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 m(Q);
    double r, p, y;
    m.getRPY(r, p, y);
    return y;
}

void MapCallback(nav_msgs::OccupancyGrid msg){
    resolution = msg.info.resolution;

    origin.Set(
        msg.info.origin.position.x,
        msg.info.origin.position.y,
        getYawFromQuaternion(msg.info.origin.orientation)
    );

    height = msg.info.height;
    width = msg.info.width;

    data.resize(height);
    for (int j = 0; j < height; j++)
    {
        data[j].resize(width);
    }

    for (int j = 0; j < height; j++)
    {
        for (int i = 0; i < width; i++)
        {
            data[j][i] = msg.data[i + width * j];
        }
    }
    
    checkForSpin = true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "global_planner_node");
    ros::NodeHandle nh;
    ROS_INFO("Node %s Started.", ros::this_node::getName().c_str());

    ros::Subscriber map_sub = nh.subscribe("/map", 1, MapCallback);
    ROS_INFO("Creating map subscriber Completed.");

    checkForSpin = false;

    while (!checkForSpin)
    {
        ROS_INFO("Waiting For map data");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("Subscribed to map topic.");
    
    navigation_pkg::Vector2 gridWorldSize(width * resolution, height * resolution);
    geometry_msgs::Point worldBottomLeft;
    worldBottomLeft.x = origin.x;
    worldBottomLeft.y = origin.y;
    worldBottomLeft.z = 0.0;

    navigation_pkg::AStar astar(gridWorldSize, resolution/2.0, worldBottomLeft, data);
    ros::spin();
    return 0;
}
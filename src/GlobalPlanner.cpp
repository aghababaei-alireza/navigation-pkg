#include <ros/ros.h>
#include <navigation_pkg/AStar.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>

double resolution;
navigation_pkg::Vector3 origin;
int height, width;
std::vector<std::vector<int> > data;

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

    origin.x = msg.info.origin.position.x;
    origin.y = msg.info.origin.position.y;
    origin.z = getYawFromQuaternion(msg.info.origin.orientation);

    height = msg.info.height;
    width = msg.info.width;

    data.resize(height);
    std::string s = "";
    for (int i = 0; i < height; i++)
    {
        data[i].resize(width);
        for (int j = 0; j < width; j++)
        {
            data[i][j] = msg.data[(i * width) + j];
            s.append(std::to_string(data[i][j]) + "\t");
        }
        s.append("\n");
    }
    ROS_INFO("map data:\n%s", s.c_str());
    
    checkForSpin = true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "global_planner_node");
    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe("/map", 1, MapCallback);

    checkForSpin = false;

    while (!checkForSpin)
    {
        ROS_INFO("Waiting For map data");
        ros::spinOnce();
    }
    
    navigation_pkg::Vector2 gridWorldSize(width * resolution, height * resolution);
    geometry_msgs::Point worldBottomLeft;
    worldBottomLeft.x = origin.x;
    worldBottomLeft.y = origin.y;
    worldBottomLeft.z = 0.0;
    navigation_pkg::AStar astar(gridWorldSize, resolution/2.0, worldBottomLeft, data);

    while (nh.ok())
    {
        ROS_INFO("main Function while");
    }
    
    return 0;
}
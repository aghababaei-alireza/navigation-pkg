#include <ros/ros.h>
#include <navigation_pkg/DWA.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "Local_Planner");
    ros::NodeHandle nh;
    ROS_INFO("Node %s has been started.", ros::this_node::getName().c_str());

    navigation_pkg::DWA dwa;

    ros::spin();
}
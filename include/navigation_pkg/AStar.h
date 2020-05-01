#ifndef ASTAR_H
#define ASTAR_H

#include <ros/ros.h>
#include <navigation_pkg/Grid.h>
#include <navigation_pkg/Target.h>
#include <nav_msgs/Odometry.h>

namespace navigation_pkg
{
    class AStar{
    public:
        //Current position of the robot (x, y, z) (type double)
        Vector3 currentPos;

        //A pointer to the grid array
        Grid grid;

        //Server to call a service for finding path
        ros::ServiceServer srv;

        //Subscriber to odom topic to get the position of the robot
        ros::Subscriber sub;

        //Client to call PlanFollower service
        ros::ServiceClient client;

        //A variable to measure the processing time of the algorithm
        ros::Time _time;

        //Constructor
        AStar(Vector2 _gridWorldSize, double _nodeRad, geometry_msgs::Point _worldBottomLeft, std::vector<std::vector<int>> data);

        //Main algorithm to find the path between start and end position
        bool FindPath(Vector3 startPos, Vector3 targetPos);

        //Generate the path based on costs calculated in FindPath function
        void RetracePath(Node* startNode, Node* endNode);

        //Calculate the distance between two nodes
        double GetDistance(Node* nodeA, Node* nodeB);

        //Check if the node exists in the vector
        bool Contain(std::vector<Node*>* vect, Node* node);

        //Service Callback -- GlobalPlanner
        bool GlobalPlanCallback(navigation_pkg::Target::Request& req, navigation_pkg::Target::Response& resp);

        //Topic Callback -- odom
        void OdomCallback(nav_msgs::Odometry msg);
    };  
}; // namespace navigation_pkg


#endif
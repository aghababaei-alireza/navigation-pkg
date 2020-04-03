#ifndef ASTAR_H
#define ASTAR_H

#include <ros/ros.h>
#include <navigation_pkg/Grid.h>
#include <navigation_pkg/Target.h>
#include <nav_msgs/Odometry.h>

namespace navigation_pkg{
    class AStar{
    public:
        navigation_pkg::Vector3 currentPos;

        Grid grid;

        ros::ServiceServer srv;

        ros::Subscriber sub;

        AStar(navigation_pkg::Vector2 _gridWorldSize, double _nodeRad, geometry_msgs::Point _worldBottomLeft, std::vector<std::vector<int> > data);

        bool FindPath(navigation_pkg::Vector3 startPos, navigation_pkg::Vector3 targetPos);

        void RetracePath(Node startNode, Node endNode);

        int GetDistance(Node nodeA, Node nodeB);

        std::vector<Node>::const_iterator GetIndex(std::vector<Node> vect, Node* node);

        bool Contain(std::vector<Node> vect, Node node);

        //Service Callback
        bool GlobalPlanCallBack(navigation_pkg::Target::Request& req, navigation_pkg::Target::Response& resp);

        //Topic Callback -- Odom
        void OdomCallBack(nav_msgs::Odometry msg);
    };
};

#endif
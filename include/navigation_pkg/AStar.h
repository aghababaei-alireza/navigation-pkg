#ifndef ASTAR_H
#define ASTAR_H

#include <ros/ros.h>
#include <navigation_pkg/Grid.h>
#include <navigation_pkg/target.h>

namespace navigation_pkg{
    class AStar{
    public:
        Grid grid;

        ros::ServiceServer srv;
        ros::NodeHandle nh;

        AStar(navigation_pkg::Vector2 _gridWorldSize, double _nodeRad, geometry_msgs::Point _worldBottomLeft);

        void FindPath(navigation_pkg::Vector3 startPos, navigation_pkg::Vector3 targetPos);

        void RetracePath(Node startNode, Node endNode);

        int GetDistance(Node nodeA, Node nodeB);

        std::vector<Node>::iterator GetIndex(std::vector<Node> vect, Node node);

        bool Contain(std::vector<Node> vect, Node node);

        bool GlobalPlanCallBack(navigation_pkg::target::Request& req, navigation_pkg::target::Response& resp);
    };
}

#endif
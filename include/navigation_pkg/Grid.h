#ifndef GRID_H
#define GRID_H

#include <ros/ros.h>
#include <navigation_pkg/Vector2.h>
#include <navigation_pkg/Vector3.h>
#include <navigation_pkg/Node.h>
#include <geometry_msgs/Point.h>

namespace navigation_pkg{
    class Grid{
    public:
        //Grid Size
        navigation_pkg::Vector2 gridWorldSize;
        double nodeRadius;
        navigation_pkg::Node** grid;
        navigation_pkg::Vector3 worldBottomLeft;

        double nodeDiameter;

        //Number of nodes in x and y dirction
        int gridSizeX, gridSizeY;

        //Generated path will be stored here
        std::vector<Vector3> path;

        //Constructor
        Grid(navigation_pkg::Vector2 _gridWorldSize, double _nodeRad, geometry_msgs::Point _worldBottomLeft, std::vector<std::vector<int> > data);

        //
        void CreateGrid(std::vector<std::vector<int> > data);

        std::vector<Node> GetNeighbours(Node node);

        Node NodeFromWorldPoint(navigation_pkg::Vector3 worldPosition);
    };
};

#endif
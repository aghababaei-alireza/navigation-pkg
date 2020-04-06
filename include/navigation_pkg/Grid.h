#ifndef GRID_H
#define GRID_H

#include <ros/ros.h>
#include <navigation_pkg/Vector2.h>
#include <navigation_pkg/Node.h>
#include <navigation_pkg/Vector3.h>
#include <geometry_msgs/Point.h>

namespace navigation_pkg{
    
    class Grid{
    public:
        //Size of the grid (type double)
        Vector2 gridWorldSize;

        //Radius of each node(pixel)
        double nodeRadius;

        //Diameter of each node(pixel)
        double nodeDiameter;

        //A 2D array of node pointers to create the grid
        Node** grid;

        //Position of the lower-left point of the map (x, y, z) (type double)
        Vector3 worldBottomLeft;

        //Number of nodes in x direction (2nd index)
        int gridSizeX;

        //Number of nodes in y direction (1st index)
        int gridSizeY;

        //Generated path will be stored here
        std::vector<Vector3> path;

        //Constructor
        Grid(Vector2 _gridWorldSize, double _nodeRad, geometry_msgs::Point _worldBottomLeft, std::vector<std::vector<int>> data);

        //Create the grid array and initialize the nodes
        void CreateGrid(std::vector<std::vector<int>> data);

        //Find the neighbours of the passed node
        std::vector<Node*> GetNeighbours(Node* node);

        //Find the node by its position
        Node* NodeFromWorldPoint(Vector3 worldPosition);

        //Find the node by its indices
        Node* NodeFromIndex(int gridX, int gridY);

    };

}; // namespace navigation_pk


#endif
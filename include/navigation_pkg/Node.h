#ifndef NODE_H
#define NODE_H

#include <ros/ros.h>
#include <navigation_pkg/Vector3.h>

namespace navigation_pkg{
    class Node{
    public:
        //True if this node is free false if occupied with an obstacle.
        bool walkable;

        //Position of the node (x, y, z) (type double)
        Vector3 worldPosition;

        //index of the node in the grid array (along x direction - 2nd index)
        int gridX;

        //index of the node in the grid array (along y direction - 1st index)
        int gridY;

        //Cost of the path from startNode to this node
        double gCost;

        //Cost of the path from this node to TargetNode
        double hCost;

        //Total cost of path through this node (= gCost + hCost)
        double fCost();

        //index of the parent of this node in grid array (along x direction - 2nd index)
        int parentX;

        //index of the parent of this node in grid array (along y direction - 1st index)
        int parentY;

        //Default Constructor
        Node();

        //Constructor
        Node(bool _walkable, Vector3 _worldPos, int _gridX, int gridY);

        //Print the information of this node
        std::string Print();

        bool operator == (Node node);
        bool operator == (Node* node);
        bool operator != (Node node);
        bool operator != (Node* node);

    private:
    };
};

#endif
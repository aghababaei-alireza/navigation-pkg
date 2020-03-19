#ifndef NODE_H
#define NODE_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

namespace navigation_pkg{
	class Node{
	    public:
		bool walkable;
		//geometry_msgs::Point worldPosition;
		int gridX;
		int gridY;

		int gCost;
		int hCost;

		int fCost();

		Node(bool _walkable, int _gridX, int _gridY);
	    private:
	};
}
#endif

#ifndef NODE_H
#define NODE_H

#include <ros/ros.h>
#include <navigation_pkg/Vector3.h>

namespace navigation_pkg{
	class Node{
	public:
		bool walkable;
		navigation_pkg::Vector3 worldPosition;
		
		//index of the node in the grid array
		int gridX, gridY;

		double gCost;
		double hCost;

		navigation_pkg::Node* parent;

		double fCost();

		Node();
		Node(bool _walkable,navigation_pkg::Vector3 _worldPos, int _gridX, int _gridY);

		std::string Print();

		bool operator == (Node node);
		bool operator != (Node node);

	private:
	};
};
#endif

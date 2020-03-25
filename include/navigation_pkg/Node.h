#ifndef NODE_H
#define NODE_H

#include <ros/ros.h>
#include <navigation_pkg/Vector3.h>

namespace navigation_pkg{
	class Node{
	public:
		bool walkable;
		navigation_pkg::Vector3 worldPosition;
		int gridX;
		int gridY;

		int gCost;
		int hCost;

		navigation_pkg::Node* parent;

		int fCost();

		Node();
		Node(bool _walkable,navigation_pkg::Vector3 _worldPos, int _gridX, int _gridY);

		bool operator == (Node node);
		bool operator != (Node node);

	private:
	};
}
#endif

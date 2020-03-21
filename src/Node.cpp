#include <navigation_pkg/Node.h>

namespace navigation_pkg{
	
	Node::Node(bool _walkable, geometry_msgs::Point _worldPos, int _gridX, int _gridY){
	    walkable = _walkable;
	    worldPosition = _worldPos;
	    gridX = _gridX;
	    gridY = _gridY;
	}

	int Node::fCost(){
	    return gCost + hCost;
	}
}
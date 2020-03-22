#include <navigation_pkg/Node.h>

namespace navigation_pkg{

	Node::Node(){
		walkable = true;
	    worldPosition.x = 0.0;
		worldPosition.y = 0.0;
		worldPosition.z = 0.0;
	    gridX = 0;
	    gridY = 0;
	}
	
	Node::Node(bool _walkable, navigation_pkg::Vector3 _worldPos, int _gridX, int _gridY){
	    walkable = _walkable;
	    worldPosition = _worldPos;
	    gridX = _gridX;
	    gridY = _gridY;
	}

	int Node::fCost(){
	    return gCost + hCost;
	}
}

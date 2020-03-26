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

	bool Node::operator ==(Node node){
		if (walkable == node.walkable &&
			worldPosition.x == node.worldPosition.x && worldPosition.y == node.worldPosition.y && worldPosition.z == node.worldPosition.z &&
			gridX == node.gridX && gridY == node.gridY &&
			gCost == node.gCost && hCost == node.hCost) return true;
		else
		{
			return false;
		}
			
	}

	bool Node::operator!= (Node node){
		return !(*this == node);
	}
};

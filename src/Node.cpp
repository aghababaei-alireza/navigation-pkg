#include <navigation_pkg/Node.h>

namespace navigation_pkg{

    Node::Node(){
        walkable = true;
        worldPosition.x = 0.0;
        worldPosition.y = 0.0;
        worldPosition.z = 0.0;
        gridX = 0;
        gridY = 0;
        gCost = 0.0;
        hCost = 0.0;
        parentX = 0;
        parentY = 0;
    }

    Node::Node(bool _walkable, Vector3 _worldPos, int _gridX, int _gridY){
        walkable = _walkable;
        worldPosition = _worldPos;
        gridX = _gridX;
        gridY = _gridY;
    }

    double Node::fCost(){
        return gCost + hCost;
    }

    std::string Node::Print(){
        std::string s = "Node: Position(" + std::to_string(worldPosition.x) + ", " + std::to_string(worldPosition.y) + ")\tgridX = " + std::to_string(gridX) + "\tgridY = " + std::to_string(gridY);
        return s;
    }

    bool Node::operator == (Node node){
        if (walkable == node.walkable &&
            worldPosition == node.worldPosition &&
            gridX == node.gridX && gridY == node.gridY)
        {
            return true;
        }
        else
        {
            return false;
        }        
    }

    bool Node::operator == (Node* node){
        return (*this == *node);
    }

    bool Node::operator != (Node node){
        return !(*this == node);
    }

    bool Node::operator != (Node* node){
        return !(*this == *node);
    }

}; // namespace navigation_pkg
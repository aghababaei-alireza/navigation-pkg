#include <navigation_pkg/AStar.h>

namespace navigation_pkg{

    AStar::AStar(navigation_pkg::Vector2 _gridWorldSize, double _nodeRad, geometry_msgs::Point _worldBottomLeft, std::vector<std::vector<int> > data):
    grid(_gridWorldSize, _nodeRad, _worldBottomLeft, data){
        ros::NodeHandle nh;
        sub = nh.subscribe("/odom", 1, &AStar::OdomCallBack, this);
        srv = nh.advertiseService("/global_planner_service", &AStar::GlobalPlanCallBack, this);
        ros::spinOnce();        
    }

    void AStar::OdomCallBack(nav_msgs::Odometry msg){
        currentPos.x = msg.pose.pose.position.x;
        currentPos.y = msg.pose.pose.position.y;
        currentPos.z = msg.pose.pose.position.z;
    }

    bool AStar::GlobalPlanCallBack(navigation_pkg::Target::Request& req, navigation_pkg::Target::Response& resp){
        ros::spinOnce();

        FindPath(currentPos, req.targetPos);
        resp.success = true;
        resp.path = grid.path;
        ros::NodeHandle nh;
        return true;
    }

    void AStar::FindPath(navigation_pkg::Vector3 startPos, navigation_pkg::Vector3 targetPos){
        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);

        std::vector<Node> openSet;
        std::vector<Node> closedSet;

        openSet.push_back(startNode);

        while (openSet.size() > 0){
            Node node = openSet[0];

            for (int i = 0; i < openSet.size(); i++){
                if (openSet[i].fCost() < node.fCost() || openSet[i].fCost() == node.fCost()){
                    if (openSet[i].hCost < node.hCost){
                        node = openSet[i];
                    }
                }
                
            }
            openSet.erase(AStar::GetIndex(openSet,node));
            closedSet.push_back(node);

            if (node == targetNode)
            {
                AStar::RetracePath(startNode, targetNode);
                return;
            }

            std::vector<Node> neighbours = grid.GetNeighbours(node);
            for (int i = 0; i < neighbours.size(); i++)
            {
                if (!neighbours[i].walkable || AStar::Contain(closedSet, neighbours[i]))
                {
                    continue;
                }
                
                int newCostToNeighbour = node.gCost + AStar::GetDistance(node, neighbours[i]);
                if (newCostToNeighbour < neighbours[i].gCost || !AStar::Contain(openSet, neighbours[i]))
                {
                    neighbours[i].gCost = newCostToNeighbour;
                    neighbours[i].hCost = AStar::GetDistance(neighbours[i], targetNode);
                    neighbours[i].parent = &node;

                    if (!AStar::Contain(openSet, neighbours[i]))
                    {
                        openSet.push_back(neighbours[i]);
                    }
                    
                }
                
                
            }
            
                    
            
        }
        
    }

    void AStar::RetracePath(Node startNode, Node endNode){
        std::vector<Vector3> path;
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            path.push_back(currentNode.worldPosition);
            currentNode = *(currentNode.parent);
        }

        std::reverse(path.begin(), path.end());
        grid.path = path;
        
    }

    int GetDistance(Node nodeA, Node nodeB){
        return sqrt(pow((nodeA.worldPosition.x - nodeB.worldPosition.x), 2) + pow((nodeA.worldPosition.y - nodeB.worldPosition.y), 2));
    }

    std::vector<Node>::iterator GetIndex(std::vector<Node> vect, Node node){

        std::vector<Node>::iterator it = vect.begin();
        for (int i = 0; i < vect.size(); i++)
            if (node == vect[i]){
                break;
            }
            else{
                it++;
            }
        return it;  
    }

    bool Contain(std::vector<Node> vect, Node node){
        for (int i = 0; i < vect.size(); i++)
        {
            if (node == vect[i])
            {
                return true;
            } 
        }
        return false;   
    }
};
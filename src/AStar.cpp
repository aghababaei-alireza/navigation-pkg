#include <navigation_pkg/AStar.h>

namespace navigation_pkg{

    AStar::AStar(navigation_pkg::Vector2 _gridWorldSize, double _nodeRad, geometry_msgs::Point _worldBottomLeft, std::vector<std::vector<int> > data):
    grid(_gridWorldSize, _nodeRad, _worldBottomLeft, data){
        //ROS_INFO("AStar Created");
        //ROS_INFO("gridWorldSize(x = %.2f, y = %.2f)\tgridSize(X = %d, Y = %d)\tNodeRadius = %.2f\tworldBottomLeft(x = %.2f, y = %.2f)", grid.gridWorldSize.x, grid.gridWorldSize.y, grid.gridSizeX, grid.gridSizeY, grid.nodeRadius, grid.worldBottomLeft.x, grid.worldBottomLeft.y);
        ros::NodeHandle nh;
        sub = nh.subscribe("/odom", 1, &AStar::OdomCallBack, this);
        srv = nh.advertiseService("/global_planner_service", &AStar::GlobalPlanCallBack, this);
        ros::spin();
    }

    void AStar::OdomCallBack(nav_msgs::Odometry msg){
        //ROS_INFO("Entered Odom Callback");
        currentPos.x = msg.pose.pose.position.x;
        currentPos.y = msg.pose.pose.position.y;
        currentPos.z = msg.pose.pose.position.z;
    }

    bool AStar::GlobalPlanCallBack(navigation_pkg::Target::Request& req, navigation_pkg::Target::Response& resp){
        ROS_INFO("Entered Service Callback");
        ros::spinOnce();

        FindPath(currentPos, req.targetPos);
        resp.success = true;
        resp.path = grid.path;
        ros::NodeHandle nh;
        return true;
    }

    void AStar::FindPath(navigation_pkg::Vector3 startPos, navigation_pkg::Vector3 targetPos){
        ROS_INFO("Entered FindPath");
        ROS_INFO("StartPos(%.3f, %.3f, %.3f)\tTargetPos(%.3f, %.3f, %.3f)", startPos.x, startPos.y, startPos.z, targetPos.x, targetPos.y, targetPos.z);
        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);

        std::vector<Node> openSet;
        std::vector<Node> closedSet;

        openSet.push_back(startNode);

        while (openSet.size() > 0){
            ROS_INFO("Entered While, openSet size(%d)", (int)openSet.size());
            Node node = openSet[0];
            ROS_INFO("OpenSet[0]: gCost=%.3f\t hCost=%.3f\tgridX=%d\tgridY=%d\twalkable=%s\tworldPosition(%.3f, %.3f)", openSet[0].gCost, openSet[0].hCost, openSet[0].gridX, openSet[0].gridY, openSet[0].walkable==true?"True":"False", openSet[0].worldPosition.x, openSet[0].worldPosition.y);

            for (int i = 0; i < openSet.size(); i++){
                if (openSet[i].fCost() < node.fCost() || openSet[i].fCost() == node.fCost()){
                    if (openSet[i].hCost < node.hCost){
                        node = openSet[i];
                    }
                }
            }

            ROS_INFO("node: gCost=%.3f\t hCost=%.3f\tgridX=%d\tgridY=%d\twalkable=%s\tworldPosition(%.3f, %.3f)", node.gCost, node.hCost, node.gridX, node.gridY, node.walkable==true?"True":"False", node.worldPosition.x, node.worldPosition.y);


            openSet.erase(AStar::GetIndex(openSet,node));
            ROS_INFO("OpenSet size(%d)", (int)openSet.size());
            closedSet.push_back(node);
            ROS_INFO("ClosedSet size(%d)", (int)closedSet.size());

            if (node == targetNode)
            {
                AStar::RetracePath(startNode, targetNode);
                return;
            }

            std::vector<Node> neighbours = grid.GetNeighbours(node);
            ROS_INFO("Neighbours Size(%d)", (int)neighbours.size());

            for (int i = 0; i < neighbours.size(); i++)
            {
                ROS_INFO("Neighbour[%d]: Pos(%.3f, %.3f)\twalkable(%s)", i, neighbours[i].worldPosition.x, neighbours[i].worldPosition.y, neighbours[i].walkable?"True":"False");
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
        ROS_INFO("Finished FindPath");
        
    }

    void AStar::RetracePath(Node startNode, Node endNode){
        ROS_INFO("Entered Retraced Path");
        std::vector<Vector3> path;
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            path.push_back(currentNode.worldPosition);
            currentNode = *(currentNode.parent);
        }

        std::reverse(path.begin(), path.end());
        grid.path = path;
        ROS_INFO("Finished Retraced Path");
        
    }

    int AStar::GetDistance(Node nodeA, Node nodeB){
        return sqrt(pow((nodeA.worldPosition.x - nodeB.worldPosition.x), 2) + pow((nodeA.worldPosition.y - nodeB.worldPosition.y), 2));
    }

    std::vector<Node>::iterator AStar::GetIndex(std::vector<Node> vect, Node node){

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

    bool AStar::Contain(std::vector<Node> vect, Node node){
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
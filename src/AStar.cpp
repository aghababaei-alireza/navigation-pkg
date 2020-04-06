#include <navigation_pkg/AStar.h>
#include <navigation_pkg/Pose.h>

namespace navigation_pkg
{
    AStar::AStar(Vector2 _gridWorldSize, double _nodeRad, geometry_msgs::Point _worldBottomLeft, std::vector<std::vector<int>> data)
    :grid(_gridWorldSize, _nodeRad, _worldBottomLeft, data)
    {
        ros::NodeHandle nh;
        sub = nh.subscribe("/odom", 1, &AStar::OdomCallback, this);
        srv = nh.advertiseService("/global_planner_service", &AStar::GlobalPlanCallback, this);
        client = nh.serviceClient<navigation_pkg::Pose>("/plan_follower");
    }

    void AStar::OdomCallback(nav_msgs::Odometry msg){
        currentPos.Set(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        );
    }

    bool AStar::GlobalPlanCallback(navigation_pkg::Target::Request& req, navigation_pkg::Target::Response& resp){
        ros::spinOnce();

        bool success = AStar::FindPath(currentPos, req.targetPos);
        resp.success = success;
        resp.path = grid.path;
        return success;
    }

    bool AStar::FindPath(Vector3 startPos, Vector3 targetPos){
        bool success = false;

        Node* startNode = grid.NodeFromWorldPoint(startPos);
        Node* targetnode = grid.NodeFromWorldPoint(targetPos);

        ROS_INFO("StartNode  => %s", startNode->Print().c_str());
        ROS_INFO("TargetNode => %s", targetnode->Print().c_str());

        std::vector<Node*> openSet;
        std::vector<Node*> closedSet;

        openSet.push_back(startNode);
        int i = 0;

        ROS_INFO("OpenSet Size(%d)", (int)openSet.size());
        while (openSet.size() > 0)
        {
            ROS_INFO("Entered While.");
            Node* node = openSet[0];
            ROS_INFO("openSet[0]\t=> %s", openSet[0]->Print().c_str());

            /**************************************/
            /* Finding the node with minimum cost */
            /**************************************/
            for (std::vector<Node*>::iterator it = openSet.begin(); it != openSet.end(); it++)
            {
                if ((*it)->fCost() < node->fCost() || (*it)->fCost() == node->fCost())
                {
                    if ((*it)->hCost < node->hCost)
                    {
                        node = *it;
                    }
                }
            }
            ROS_INFO("node\t\t=> %s", node->Print().c_str());
            
            /*****************************************************************************/
            /* Removing the node (with minimum cost) from openSet and adding to closedSet*/
            /*****************************************************************************/
            for (std::vector<Node*>::iterator it = openSet.begin(); it < openSet.end(); it++)
            {
                if (*it == node)
                {
                    openSet.erase(it);
                    break;
                }
            }
            ROS_INFO("OpenSet Size(%d)", (int)openSet.size());
            closedSet.push_back(node);
            ROS_INFO("ClosedSet Size(%d)", (int)closedSet.size());
            
            /***********************************/
            /* Checking if we reach the target */
            /***********************************/
            if (node == targetnode)
            {
                ROS_INFO("Reached the target. Retracing the path.");
                success = true;
                AStar::RetracePath(startNode, targetnode);
                return success;
            }

            ROS_INFO("Checking neighbours.");
            /************************************************************/
            /* Get the neighbours of the node and calculate their costs */
            /************************************************************/
            std::vector<Node*> neighbours = grid.GetNeighbours(node);
            ROS_INFO("neighbours size(%d)", (int)neighbours.size());
            for (std::vector<Node*>::iterator it = neighbours.begin(); it < neighbours.end(); it++)
            {
                ROS_INFO("Checking each neighbour.");
                if (!(*it)->walkable || AStar::Contain(&closedSet, *it)) continue;
                
                ROS_INFO("Checking each prepared neighbour.");
                double newCostToNeighbour = node->gCost + AStar::GetDistance(node, *it);
                if (newCostToNeighbour < (*it)->gCost || !AStar::Contain(&openSet, *it))
                {
                    (*it)->gCost = newCostToNeighbour;
                    (*it)->hCost = AStar::GetDistance((*it), targetnode);
                    (*it)->parentX = node->gridX;
                    (*it)->parentY = node->gridY;
                    if (!AStar::Contain(&openSet, *it))
                    {
                        openSet.push_back(*it);
                    }
                    ROS_INFO("Neighbour: %s", (*it)->Print().c_str());
                }
                
            }
            i++;
        }
        ROS_INFO("Finished FindPath, %s", success ? "Successfully" : "Failed");
        return success;
    }

    void AStar::RetracePath(Node* startNode, Node* endNode){
        ROS_INFO("Entered Retraced Path");
        std::vector<Vector3> path;
        Node* currentNode = endNode;

        while (currentNode != startNode)
        {
            path.push_back(currentNode->worldPosition);
            currentNode = grid.NodeFromIndex(currentNode->parentX, currentNode->parentY);
        }

        std::reverse(path.begin(), path.end());
        grid.path = path;

        std::vector<geometry_msgs::Pose> pose;
        pose.resize(path.size());
        navigation_pkg::Pose msg;
        for (int i = 0; i < path.size(); i++)
        {
            pose[i].position.x = path[i].x;
            pose[i].position.y = path[i].y;
            pose[i].position.z = path[i].z;
        }
        msg.request.pose = pose;
        client.call(msg);
    }

    double AStar::GetDistance(Node* nodeA, Node* nodeB){
        double dist = pow((nodeA->worldPosition.x - nodeB->worldPosition.x), 2) + pow((nodeA->worldPosition.y - nodeB->worldPosition.y), 2) + pow((nodeA->worldPosition.z - nodeB->worldPosition.z), 2); 
        return sqrt(dist);
    }

    bool AStar::Contain(std::vector<Node*>* vect, Node* node){
        for (std::vector<Node*>::iterator it = vect->begin(); it < vect->end(); it++)
        {
            if (node == (*it))
            {
                return true;
            }
        }
        return false;
    }

}; // namespace navigation_pkg

#include <navigation_pkg/Grid.h>
#include <cmath>

namespace navigation_pkg{
    Grid::Grid(navigation_pkg::Vector2 _gridWorldSize, double _nodeRad, geometry_msgs::Point _worldBottomLeft, std::vector<std::vector<int> > data)
        :nodeRadius(_nodeRad), gridWorldSize(_gridWorldSize), worldBottomLeft(_worldBottomLeft.x, _worldBottomLeft.y, _worldBottomLeft.z) {
        ROS_INFO("Entered Grid Constructor");
        nodeDiameter = nodeRadius * 2.0;

        gridSizeX = (int) round(gridWorldSize.x / nodeDiameter);
        gridSizeY = (int) round(gridWorldSize.y / nodeDiameter);

        CreateGrid(data);
    }

    void Grid::CreateGrid(std::vector<std::vector<int> > data){
        //Creating a 2D array of Nodes
        grid = new Node*[gridSizeX];
        for (int i = 0; i < gridSizeX; i++){
            grid[i] = new Node[gridSizeY];
            for (int j = 0; j < gridSizeY; j++){
                navigation_pkg::Vector3 worldPoint(
                    worldBottomLeft.x + ((i * nodeDiameter) + nodeRadius),
                    worldBottomLeft.y + ((j * nodeDiameter) + nodeRadius),
                    worldBottomLeft.z
                );

                bool walkable = data[i][j]==0 ? true : false; //TODO : CHECK
                grid[i][j].walkable = walkable;
                grid[i][j].worldPosition = worldPoint;
                grid[i][j].gridX = i;
                grid[i][j].gridY = j;
            }
        }
        

        ROS_INFO("map data size(%d, %d)", (int)data.size(), (int)data[0].size());
        

    }

    std::vector<Node> Grid::GetNeighbours(Node node){
        std::vector<Node> neighbours;
        for (int i = -1; i <= 1; i++){
            for (int j = -1; j <= 1; j++){
                if (i == 0 && j == 0) continue;

                int checkX = node.gridX + i;
                int checkY = node.gridY + j;

                if (checkX >= 0 && checkX < gridSizeX && checkY >= 0 && checkY < gridSizeY)
                {
                    neighbours.push_back(grid[checkX][checkY]);
                }
                
            }
            
        }
        return neighbours;
    }

    Node Grid::NodeFromWorldPoint(navigation_pkg::Vector3 worldPosition){
        /*
        double percentX = (worldPosition.x + gridWorldSize.x/2) / gridWorldSize.x;
        double percentY = (worldPosition.y + gridWorldSize.y/2) / gridWorldSize.y;

        int x = (int)round((gridSizeX-1) * percentX);
        int y = (int)round((gridSizeY-1) * percentY);
        */

        double percentX = (worldPosition.x - worldBottomLeft.x) / gridWorldSize.x;
        double percentY = (worldPosition.y - worldBottomLeft.y) / gridWorldSize.y;

        int x = (int)round((gridSizeX-1) * percentX);
        int y = (int)round((gridSizeY-1) * percentY);
        
        return grid[x][y];
    }
};
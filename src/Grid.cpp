#include <navigation_pkg/Grid.h>
#include <fstream>
#include <iostream>

namespace navigation_pkg
{

    Grid::Grid(Vector2 _gridWorldSize, double _nodeRad, geometry_msgs::Point _worldBottomLeft, std::vector<std::vector<int>> data)
    :nodeRadius(_nodeRad),
    gridWorldSize(_gridWorldSize),
    worldBottomLeft(_worldBottomLeft.x, _worldBottomLeft.y, _worldBottomLeft.z)
    {
        ROS_INFO("Entered Grid Constructor");
        nodeDiameter = 2.0 * nodeRadius;

        gridSizeX = (int) round(gridWorldSize.x / nodeDiameter);
        gridSizeY = (int) round(gridWorldSize.y / nodeDiameter);

        Grid::CreateGrid(data);
    }

    void Grid::CreateGrid(std::vector<std::vector<int>> data){
        ROS_INFO("Entered CreateGrid");
        //Creating 2D array of node pointers
        grid = new Node*[gridSizeY];
        ROS_INFO("Grid Resize(height)");
        for (int j = 0; j < gridSizeY; j++)
        {
            grid[j] = new Node[gridSizeX];
        }
        ROS_INFO("Grid Resize(width)");

        for (int j = 0; j < gridSizeY; j++)
        {
            for (int i = 0; i < gridSizeX; i++)
            {
                Vector3 worldPoint(
                    worldBottomLeft.x + (i * nodeDiameter) + nodeRadius,
                    worldBottomLeft.y + (j * nodeDiameter) + nodeRadius,
                    worldBottomLeft.z
                );

                Node* n = &grid[j][i];
                bool _walkable = data[j][i] == 0 ? true : false;
                // ROS_INFO("Start Assigning data to nodes.");
                grid[j][i].walkable = _walkable;
                grid[j][i].worldPosition = worldPoint;
                grid[j][i].gridX = i;
                grid[j][i].gridY = j;
                // ROS_INFO("grid[%d][%d] =>\t%s", j, i, n->Print().c_str());
            }
        }
        ROS_INFO("Grid Construction Completed.");
    }

    std::vector<Node*> Grid::GetNeighbours(Node* node){
        std::vector<Node*> neighbours;
        for (int j = -1; j <= 1; j++)
        {
            for (int i = -1; i <= 1; i++)
            {
                if (i == 0 && j == 0) continue;
                int checkX = node->gridX + i;
                int checkY = node->gridY + j;
                if (checkX >= 0 && checkX < gridSizeX && checkY >= 0 && checkY < gridSizeY)
                {
                    neighbours.push_back(&grid[checkY][checkX]);
                }
                
            }
            
        }
        return neighbours;
    }

    Node* Grid::NodeFromWorldPoint(Vector3 worldPosition){
        double percentX = (worldPosition.x - worldBottomLeft.x) / gridWorldSize.x;
        double percentY = (worldPosition.y - worldBottomLeft.y) / gridWorldSize.y;

        int x = (int) round((gridSizeX-1) * percentX);
        int y = (int) round((gridSizeY-1) * percentY);

        return &grid[y][x];
    }

    Node* Grid::NodeFromIndex(int _gridX, int _gridY){
        return &grid[_gridY][_gridX];
    }

    void Grid::SavePathToFile(){
        std::ofstream f;
        f.open("map.txt");
        if (f.fail())
        {
            ROS_ERROR("Error opening file.");
            return;
        }
        
        for (int j = gridSizeY-1; j >= 0; j--)
        {
            for (int i = 0; i < gridSizeX; i++)
            {
                if (!grid[j][i].walkable)
                {
                    f.put('0');
                }
                else
                {
                    if (Grid::IsInPath(grid[j][i].worldPosition))
                    {
                        f.put('*');
                    }
                    else
                    {
                        f.put('-');
                    }
                }
                
            }
            f.put('\n');
        }
        f.close();
    }

    bool Grid::IsInPath(Vector3 v){
        for (std::vector<Vector3>::iterator it = path.begin(); it != path.end(); it++)
        {
            if ((*it) == v)
            {
                return true;
            }
        }
        return false;
    }

}; // namespace navigation_pkg

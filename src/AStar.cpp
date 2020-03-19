#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <navigation_pkg/Node.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "node");
	ros::NodeHandle nh;
	
	navigation_pkg::Node n(true, 10, 20);
	ROS_INFO("n: walkable = %s, worldPos(), gridX = %d, gridY = %d", n.walkable?"True":"False", n.gridX, n.gridY);

	ros::spin();
}

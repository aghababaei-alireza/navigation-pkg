#include <ros/ros.h>
#include <navigation_pkg/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#define VEL 1.0       /*  m/s */
#define EPS 1.0e-3    /*  m   */

double xPos = 0.0, yPos = 0.0, zPos = 0.0;
double xDes, yDes, zDes;

ros::ServiceServer server;
ros::Subscriber sub;
ros::Publisher pub;

double Distance(geometry_msgs::Point p1, geometry_msgs::Point p2){
    ROS_INFO("Current:\tX = %f\tY = %f\tZ = %f\nDesired:\tX = %f\tY = %f\tZ = %f", p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
    return sqrt(pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2) + pow((p1.z - p2.z),2) );
}

bool srv_callback(navigation_pkg::Pose::Request& req, navigation_pkg::Pose::Response& resp){
    xDes = req.pose.position.x;
    yDes = req.pose.position.y;
    zDes = req.pose.position.z;
    while (true)
    {
        geometry_msgs::Point current;
        current.x = xPos;
        current.y = yPos;
        current.z = zPos;
        geometry_msgs::Point Desired = req.pose.position;
        
        geometry_msgs::Twist vel;
        double d = Distance(current, Desired);
        ROS_INFO("Dist = %f", d);
        if (d < EPS)
        {
            vel.linear.x = 0.0;
            vel.linear.y = 0.0;
            vel.linear.z = 0.0;
            vel.angular.x = 0.0;
            vel.angular.y = 0.0;
            vel.angular.z = 0.0;
            pub.publish(vel);
            resp.success = true;
            break;
        }
                
        vel.linear.x = VEL;
        vel.linear.y = 0.0;
        vel.linear.z = 0.0;
        vel.angular.x = 0.0;
        vel.angular.y = 0.0;
        vel.angular.z = 0.0;
        
        pub.publish(vel);
        
    }
    return true;
}



void pose_callback(nav_msgs::Odometry data){
    xPos = data.pose.pose.position.x;
    yPos = data.pose.pose.position.y;
    zPos = data.pose.pose.position.z;
    //ROS_INFO("x:%f, y:%f, z:%f", xPos, yPos, zPos);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "reachGoal");
    ros::NodeHandle nh;
    ROS_INFO("'%s' node has been started.",ros::this_node::getName().c_str());
    
    server = nh.advertiseService("/moveToGoal", srv_callback);
    sub = nh.subscribe("/odom", 10, pose_callback);
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    ros::spin();

    
    return 0;
}
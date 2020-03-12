#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>

#define K_LIN 0.5
#define K_ANG 0.5


double x, y, theta;

ros::Subscriber sub;
ros::Publisher pub;

void newOdom(const nav_msgs::Odometry data){
    x = data.pose.pose.position.x;
    y = data.pose.pose.position.y;
    geometry_msgs::Quaternion q = data.pose.pose.orientation;
    tf::Quaternion Q(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 m(Q);
    double roll, pitch;
    m.getRPY(roll, pitch, theta);
    ROS_INFO("ODOM CALLBACK: Current position: x = %.2f, y = %.2f, theta = %.2f", x, y, theta);
}

double Distance(geometry_msgs::Point start, geometry_msgs::Point end){
    return sqrt(pow((start.x - end.x),2) + pow((start.y - end.y),2) + pow((start.z - end.z),2));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Go_To_Goal");
    ros::NodeHandle nh;
    ROS_INFO("The node %s has been started.", ros::this_node::getName().c_str());
    sub = nh.subscribe("/odom", 1, &newOdom);
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ROS_INFO("BEFORE FIRST ROS::SPINONCE()");
    ros::spinOnce();
    ROS_INFO("AFTER FIRST ROS::SPINONCE()");

    geometry_msgs::Twist speed;

    ros::Rate r(10);

    geometry_msgs::Point goal, current;
    goal.x = -1;
    goal.y = 2;
    goal.z = 0;

    
    int i = 0;

    while (nh.ok())
    {
        current.x = x;
        current.y = y;
        current.z = 0;

        double x_inc = goal.x - current.x;
        double y_inc = goal.y - current.y;
        double d = Distance(current, goal);
        ROS_INFO("%%%%%%%%   TRY %2d   %%%%%%%%", ++i);
        ROS_INFO("Current position: x = %.2f, y = %.2f, theta = %.2f", current.x, current.y, theta);
        ROS_INFO("Goal Position: x = %.2f y = %.2f", goal.x, goal.y);
        ROS_INFO("Distance = %.2f", d);
        
        if (d > 5.0e-2){
            
            double angle = atan2(y_inc, x_inc);
            ROS_INFO("Angle = %.2f", angle);
            /*if(abs(angle - theta) > 0.1){
                speed.linear.x = 0.0;
                speed.linear.y = 0.0;
                speed.linear.z = 0.0;
                speed.angular.x = 0.0;
                speed.angular.y = 0.0;
                speed.angular.z = 0.3;
                ROS_INFO("Velocity: x = %.2f, theta = %.2f", speed.linear.x, speed.angular.z);
                pub.publish(speed);
            }
            else{
                speed.linear.x = 0.5;
                speed.linear.y = 0.0;
                speed.linear.z = 0.0;
                speed.angular.x = 0.0;
                speed.angular.y = 0.0;
                speed.angular.z = 0.0;
                ROS_INFO("Velocity: x = %.2f, theta = %.2f", speed.linear.x, speed.angular.z);
                pub.publish(speed);
            }*/

            speed.linear.x = (d * K_LIN <= 0.5) ? d * K_LIN : 0.5;
            speed.linear.y = 0.0;
            speed.linear.z = 0.0;
            speed.angular.x = 0.0;
            speed.angular.y = 0.0;
            speed.angular.z = ((angle - theta) * K_ANG <= 0.5) ? (angle - theta) * K_ANG : 0.5;
            ROS_INFO("Velocity: x = %.2f, theta = %.2f", speed.linear.x, speed.angular.z);
            pub.publish(speed);
        }
        else
        {
            ROS_INFO("Reached the goal.");
            speed.linear.x = 0.0;
            speed.linear.y = 0.0;
            speed.linear.z = 0.0;
            speed.angular.x = 0.0;
            speed.angular.y = 0.0;
            speed.angular.z = 0.0;
            pub.publish(speed);
        }

        ROS_INFO("%%%%%%%% END OF TRY %%%%%%%%");

        ros::spinOnce();
        r.sleep();
        
    }
    


    return 0;
}

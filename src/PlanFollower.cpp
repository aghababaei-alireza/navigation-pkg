#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <navigation_pkg/Pose.h>
#include <tf/tf.h>


#define K_LIN 0.5
#define K_ANG 0.5

//A subscriber to get the current position of the robot (/odom topic)
ros::Subscriber pose_sub;

//A publisher to publish velocity command to move the robot (/cmd_vel topic)
ros::Publisher vel_pub;

//A service to Follow the plan
ros::ServiceServer follow_plan_srv;

struct position2D
{
    //x position
    double x;
    //y position
    double y;
    //Orientation (YAW)
    double theta;
};

//Struct for saving the current position of the robot
position2D current;

//Struct for Saving the goal position
position2D goal;

//Speed to publish to cmd_vel topic
geometry_msgs::Twist speed;




double getYawFromQuaternion(geometry_msgs::Quaternion q){
    tf::Quaternion Q(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 m(Q);
    double r, p, y;
    m.getRPY(r, p, y);
    return y;
}

double Distance(position2D start, position2D end){
    return sqrt(pow((start.x - end.x),2) + pow((start.y - end.y),2));
}

void PublishVelocity(double Lx, double Ly, double Lz, double Ax, double Ay, double Az){
    speed.linear.x = Lx;
    speed.linear.y = Ly;
    speed.linear.z = Lz;
    speed.angular.x = Ax;
    speed.angular.y = Ay;
    speed.angular.z = Az;
    vel_pub.publish(speed);
}


//Callback function for pose_sub subscriber. Saves the current position of the robot to the local variables.
void pose_Callback(const nav_msgs::Odometry::ConstPtr msg){
    current.x = msg->pose.pose.position.x;
    current.y = msg->pose.pose.position.y;
    current.theta = getYawFromQuaternion(msg->pose.pose.orientation);
}

bool service_Callback(navigation_pkg::Pose::Request &req, navigation_pkg::Pose::Response &resp){
    ros::Rate r(10);
    ROS_INFO("Path recieved. Rotating to match the orientation...");

    /****************************
    goal.x = req.pose[0].position.x;
    goal.y = req.pose[0].position.y;
    goal.theta = getYawFromQuaternion(req.pose[0].orientation);

    while (true)
    {
        ros::spinOnce();

        double x_inc = goal.x - current.x;
        double y_inc = goal.y - current.y;
        double dist = Distance(current, goal);

        double angle = atan2(y_inc, x_inc);
        if (std::abs(angle - current.theta) > 0.1)
        {
            double Az = ((angle - current.theta) * K_ANG <= 0.5) ? (angle - current.theta) * K_ANG : 0.5;
            PublishVelocity(0.0, 0.0, 0.0, 0.0, 0.0, Az);
        }
        else
        {
            break;
        }
            
    }

    ROS_INFO("Moving to the target...");
    
    /****************************/
    
    for (int i = 0; i < req.pose.size(); i++)
    {
        
        //To get the goal position
        goal.x = req.pose[i].position.x;
        goal.y = req.pose[i].position.y;
        goal.theta = getYawFromQuaternion(req.pose[i].orientation);

        while (true){
            //To get the current position of the robot
            ros::spinOnce();

            double x_inc = goal.x - current.x;
            double y_inc = goal.y - current.y;
            double dist = Distance(current, goal);

            if (dist > 5.0e-2){ 
                double angle = atan2(y_inc, x_inc);
                
                // double Ax = (dist * K_LIN <= 0.5) ? dist * K_LIN : 0.5;
                // double Az = ((angle - current.theta) * K_ANG <= 0.5) ? (angle - current.theta) * K_ANG : 0.5;
                // PublishVelocity(Ax, 0.0, 0.0, 0.0, 0.0, Az);
                if (std::abs(angle - current.theta) > 0.1)
                {
                    double Az = ((angle - current.theta) * K_ANG <= 0.5) ? (angle - current.theta) * K_ANG : 0.5;
                    PublishVelocity(0.0, 0.0, 0.0, 0.0, 0.0, Az);
                }
                else
                {
                    double Lx = (dist * K_LIN <= 0.5) ? dist * K_LIN : 0.5;
                    PublishVelocity(Lx, 0.0, 0.0, 0.0, 0.0, 0.0);
                }
                
                
            }
            else{
                break;
            }

            r.sleep();
        }

        ROS_INFO("i = %d reached.", i);
    }
    /*speed.linear.x = 0.0;
    speed.linear.y = 0.0;
    speed.linear.z = 0.0;
    speed.angular.x = 0.0;
    speed.angular.y = 0.0;
    speed.angular.z = 0.0;
    vel_pub.publish(speed);*/
    PublishVelocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    resp.success = true;
    return true;
    
}




int main(int argc, char** argv){
    ros::init(argc, argv, "Plan_Follower");
    ros::NodeHandle nh;
    
    ROS_INFO("The node %s has been started.", ros::this_node::getName().c_str());
    
    pose_sub = nh.subscribe("/odom", 1, pose_Callback);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    follow_plan_srv = nh.advertiseService("/plan_follower", service_Callback);

    

    ros::spin();
}
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <navigation_pkg/ActivateGoal.h>
#include <tf/tf.h>

#define K_LIN 0.5
#define K_ANG 0.5

const double max_v = 0.26;
const double max_w = 1.82;

const double dist_precision = 1.0e-1;

ros::Publisher vel_pub;
ros::Subscriber pose_sub;
ros::ServiceServer srv;

bool active = false;

geometry_msgs::Pose currentPos;
geometry_msgs::Twist currentVel;
geometry_msgs::Pose goalPos;
geometry_msgs::Twist speed;

enum State{
    CONTROLLING,
    REACHED
};

State _state;

bool checkPos = false;

void ChangeState(State state){
    _state = state;
    ROS_INFO("State changed to %d", _state);
}

void Pose_Callback(nav_msgs::Odometry msg){
    currentPos.position = msg.pose.pose.position;
    currentPos.orientation = msg.pose.pose.orientation;
    currentVel = msg.twist.twist;
    checkPos = true;
}

bool Go_To_Point_Switch_Callback(navigation_pkg::ActivateGoal::Request& req, navigation_pkg::ActivateGoal::Response& resp){
    active = req.activate;
    _state = CONTROLLING;
    goalPos = req.pose;
    resp.success = true;
    return true;
}

void SetSpeed(double Lx, double Az){
    speed.linear.x = Lx;
    speed.angular.z = Az;
}

double getYawFromQuaternion(geometry_msgs::Quaternion q){
    tf::Quaternion Q(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 m(Q);
    double r, p, y;
    m.getRPY(r, p, y);
    return y;
}

void Move(){
    double err_pos = sqrt(pow(currentPos.position.x-goalPos.position.x, 2) + pow(currentPos.position.y-goalPos.position.y, 2));
    double err_yaw = atan2(goalPos.position.y-currentPos.position.y, goalPos.position.x-currentPos.position.x) - getYawFromQuaternion(currentPos.orientation);
    if (err_pos > dist_precision)
    {
        double Lx = (err_pos * K_LIN > max_v) ? max_v: err_pos * K_LIN;
        double Az = (err_yaw * K_ANG > max_w) ? max_w: err_yaw * K_ANG;
        SetSpeed(Lx, Az);
        vel_pub.publish(speed);
    }
    else
    {
        ChangeState(REACHED);
    }
    
    
}

void Done(){
    SetSpeed(0.0, 0.0);
    vel_pub.publish(speed);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "Go_To_Point");
    ros::NodeHandle nh;

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pose_sub = nh.subscribe("/odom", 1, Pose_Callback);
    srv = nh.advertiseService("/Go_To_Point_Switch", Go_To_Point_Switch_Callback);

    ROS_INFO("Node %s has been started.", ros::this_node::getName().c_str());

    while(!checkPos){
        ROS_INFO("Waiting for /odom topic.");
        ros::spinOnce();
        ros::Duration(1).sleep();
    }

    ros::Rate r(20);

    while (nh.ok())
    {
        ros::spinOnce();
        if (!active) continue;
        
        switch (_state)
        {
        case CONTROLLING:
            Move();
            break;
        case REACHED:
            Done();
            break;
        
        default:
            break;
        }
    }
    


}
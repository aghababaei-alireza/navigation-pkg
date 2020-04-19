#ifndef BUG0_H
#define BUG0_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <navigation_pkg/Pose.h>
#include <tf/tf.h>
#include <navigation_pkg/ActivateGoal.h>
#include <cmath>

namespace navigation_pkg
{
    class Bug0{
    public:
        ros::Subscriber pose_sub;                               //subscriber to get the position of the robot
        ros::Subscriber laser_sub;                              //Subscriber to get the laser scanner data
        ros::Publisher vel_pub;                                 //Velocity publisher
        ros::ServiceServer local_plan_srv;                      //Service Server to start the local planning
        ros::ServiceServer shutDown_srv;                        //Service Server to stop the local planning service
        ros::ServiceClient go_to_point_client;                  //Client to call Go_To_Point service
        ros::ServiceClient follow_wall_client;                  //Client to call Follow_Wall service
        ros::NodeHandle nh;                                     //Node Handle

        double max_allow_v;                                     //Maximum allowable linear velocity specified in datasheet (m/s)
        double max_allow_w;                                     //Maximum allowable angular velocity specified in datasheet (rad/s)

        double dist_precision;

        geometry_msgs::Pose currentPos;                         //Current position and orientation of the robot
        geometry_msgs::Pose goal;                               //Goal position
        geometry_msgs::Twist currentVel;                        //Current linear and angular velocity of the robot
        geometry_msgs::Twist speed;                             //Speed to publish to the cmd_vel topic

        sensor_msgs::LaserScan laser;                           //Stores laserScanner data - distance from robot to the obstacles

        bool checkPose = false, checkLaser = false;

        enum Direction{
            R,
            FR,
            F,
            FL,
            L
        };

        enum State{
            GoToPoint,
            FollowWall
        };

        State _state;

        double regions[5];                                      //An array to store the minimum distance to obstacles in 5 regions in front of robot

        Bug0();
        bool Bug0_Callback(navigation_pkg::Pose::Request& req, navigation_pkg::Pose::Response& resp);
        void Pose_Callback(nav_msgs::Odometry msg);
        void Laser_Callback(sensor_msgs::LaserScan msg);
        double getYawFromQuaternion(geometry_msgs::Quaternion q);
        bool Bug0Algorithm(geometry_msgs::Pose& currentPos, geometry_msgs::Pose& goalPos);
        void ChangeState(State state);
        double Normalize_Angle(double angle);



    };
}; // namespace navigation_pkg


#endif
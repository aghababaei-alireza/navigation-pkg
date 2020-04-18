#ifndef DWA_H
#define DWA_H

#include <ros/ros.h>
#include <navigation_pkg/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <cmath>
#include <geometry_msgs/Pose2D.h>
#include <std_srvs/Empty.h>

namespace navigation_pkg
{
    class DWA{
    public:
        ros::ServiceServer local_plan_srv;      //Service Server to start the local planning
        ros::Subscriber pose_sub;               //subscriber to get the position of the robot
        ros::Subscriber laser_sub;              //Subscriber to get the laser scanner data
        ros::Publisher vel_pub;                 //Velocity publisher
        ros::ServiceServer shutDown_srv;        //
        ros::ServiceServer debug_srv;           //
        ros::NodeHandle nh;

        double dt;                              //Time interval (sec)     

        double dv_a;                            //Linear acceleration (m/s2)
        double dv_b;                            //Linear deceleration (m/s2)
        double dw_a;                            //Angular acceleration (rad/s2)
        double dw_b;                            //Angular decelaration (rad/s2)

        double max_allow_v;                     //Maximum allowable linear velocity specified in datasheet (m/s)
        double max_allow_w;                     //Maximum allowable angular velocity specified in datasheet (rad/s)

        double min_window_v;                    //Minimum linear velocity in dynamic window
        double max_window_v;                    //Maximum linear velocity in dynamic window
        double min_window_w;                    //Minimum angular velocity in dynamic window
        double max_window_w;                    //Maximum angular velocity in dynamic window

        int v_samples;                          //Number of samples to check for linear velocity in dynamic window
        int w_samples;                          //Number of samples to check for angular velocity in dynamic window
        double v_increment;                     //distance between two linear velocity samples in dynamic window
        double w_increment;                     //distance between two angular velocity samples in dynamic window

        geometry_msgs::Pose currentPos;         //Current position and orientation of the robot
        geometry_msgs::Pose goal;               //Goal position
        geometry_msgs::Twist currentVel;        //Current linear and angular velocity of the robot
        geometry_msgs::Twist speed;             //Speed to publish to the cmd_vel topic

        sensor_msgs::LaserScan laser;           //Stores laserScanner data - distance from robot to the obstacles

        double alpha;                           //Target heading coefficient of objective function
        double beta;                            //Clearance coefficient of objective function
        double gamma;                           //Velocity heading coefficient of objective function

        double robotWidth;                      //Width of the robot using for collision detection
        int angle_samples;                       //Number of samples to find the minimum distance to obstacles
        int line_samples;                        //Number of samples to check the collision along robot width
        double angle_increment;                 //Angular distance between two points on the arc to find minimum distance to obstacles
        double line_increment;                  //Linear distance between two points along the robot width
        double sl;                              //Look_ahead distance of used sensor

        bool checkPose, checkLaser, isShutDown;



        DWA();                                  //Constructor
        bool DWA_Callback(navigation_pkg::Pose::Request& req, navigation_pkg::Pose::Response& resp);
        void Pose_Callback(nav_msgs::Odometry msg);
        void Laser_Callback(sensor_msgs::LaserScan msg);
        void DynamicWindowApproach(geometry_msgs::Pose& currentPos, geometry_msgs::Pose& goalPos);

        double CalcDesiredVel(geometry_msgs::Pose currentPos, geometry_msgs::Pose goalPos);
        double GetDistance(geometry_msgs::Pose poseA, geometry_msgs::Pose poseB);

        void GenerateDynamicWindow(double Vc, double Wc, double& Vmin, double& Vmax, double& Wmin, double& Wmax);

        double GetMinimumDistanceToObstacle(double v, double w);

        double CalcTargetHeading(geometry_msgs::Pose currentPos, geometry_msgs::Pose goalPos, double v, double w);

        double CalcClearance(double distance, double v, double w);

        double CalcVelocity(double v, double des_v);

        void SetSpeed(double Lx, double Az);

        double getYawFromQuaternion(geometry_msgs::Quaternion q);

        geometry_msgs::Pose2D SetPose2D(double x, double y, double theta);

        double getLaserDist(double angle);

        bool ShutdownCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

        bool Debug_Callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
    };
}; // namespace navigation_pkg


#endif
#ifndef VFH_H
#define VFH_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <navigation_pkg/VFH_LocalMap.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <navigation_pkg/Pose.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>

namespace navigation_pkg
{
    class VFH{
    public:
        ros::NodeHandle nh;
        ros::Subscriber pose_sub;
        ros::Subscriber laser_sub;
        ros::Publisher vel_pub;
        ros::ServiceServer local_plan_srv;
        ros::ServiceServer shutDown_srv;

        geometry_msgs::Pose currentPos;
        geometry_msgs::Pose goalPos;
        geometry_msgs::Twist currentVel;
        geometry_msgs::Twist speed;

        sensor_msgs::LaserScan laser;

        bool checkPose, checkLaser;
        bool isShutDown;

        double target_direction;
        double steer_magnitude;
        double steer_direction;

        double POD[360];
        double smoothed_POD[360];

        VFH_LocalMap certainty_grid;

        VFH();

        void Pose_Callback(nav_msgs::Odometry msg);
        void Laser_Callback(sensor_msgs::LaserScan msg);
        bool VFH_Callback(navigation_pkg::Pose::Request& req, navigation_pkg::Pose::Response& resp);
        bool ShutdownCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

        void VectorFieldHistogram();

        void Polar_Histogram();
        void Smoothed_Polar_Histogram();
        int Data_Reduction();
        void Speed_Control(int k_c);

        void SetSpeed(double Lx, double Az);
        double getYawFromQuaternion(geometry_msgs::Quaternion q);
        double GetDistance(geometry_msgs::Pose poseA, geometry_msgs::Pose poseB);

    };
}; // namespace navigation_pkg


#endif
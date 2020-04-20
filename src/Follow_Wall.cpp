#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <navigation_pkg/ActivateGoal.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>

#define K_LIN 0.5
#define K_ANG 0.5

const double max_v = 0.26;
const double max_w = 1.82;

ros::Publisher vel_pub;
ros::Subscriber pose_sub;
ros::Subscriber laser_sub;
ros::ServiceServer srv;
ros::ServiceClient go_to_point_client;

geometry_msgs::Pose currentPos;
geometry_msgs::Twist currentVel;
geometry_msgs::Pose goalPos;
geometry_msgs::Twist speed;
sensor_msgs::LaserScan laser;
double regions[5];

bool checkPos = false, checkLaser = false;

bool active = false;

enum Direction{
    R, FR, F, FL, L
};

enum State{
    FIND_WALL,
    TURN,
    FOLLOW_WALL
};

enum Side{
    LEFT, 
    RIGHT
};

std::string state_description[3] = {"Find_Wall", "Turn", "Follow_Wall"};
std::string side_description[2] = {"Left", "Right"};

State _state;
Side _side;

void ChangeState(State state){
    _state = state;
    ROS_INFO("State changed to [%d: %s] side: %s.", _state, state_description[_state].c_str(), side_description[_side].c_str());
}

double Normalize_Angle(double angle){
    double a;
    if (std::abs(angle) > M_PI)
    {
        a = angle - (2 * M_PI * angle) / std::abs(angle);
    }
    return a;
}

void Pose_Callback(nav_msgs::Odometry msg){
    currentPos.position = msg.pose.pose.position;
    currentPos.orientation = msg.pose.pose.orientation;
    currentVel = msg.twist.twist;
    checkPos = true;
}

bool Follow_Wall_Switch_Callback(navigation_pkg::ActivateGoal::Request& req, navigation_pkg::ActivateGoal::Response& resp){
    active = req.activate;
    _state = FIND_WALL;
    goalPos = req.pose;
    resp.success = true;
    return true;
}

void FindNewState(){
    double d = 1.0;
    if (regions[F] > d && regions[FL] > d && regions[FR] > d) //case 1: Nothing
    {
        ChangeState(TURN);
    }
    else if (regions[F] < d && regions[FL] > d && regions[FR] > d) //case 2: Front
    {
        ChangeState(TURN);
    }
    else if (regions[F] > d && regions[FL] > d && regions[FR] < d) //case 3: FrontRight
    {
        // ChangeState(FOLLOW_WALL);
        if (_side == LEFT)
        {
            ChangeState(TURN);
        }
        else if (_side == RIGHT)
        {
            ChangeState(FIND_WALL);
        }
    }
    else if (regions[F] > d && regions[FL] < d && regions[FR] > d) //case 4: FrontLeft
    {
        // ChangeState(FIND_WALL);
        if (_side == LEFT)
        {
            ChangeState(FIND_WALL);
        }
        else if (_side == RIGHT)
        {
            ChangeState(TURN);
        }
    }
    else if (regions[F] < d && regions[FL] > d && regions[FR] < d) //case 5: Front & FrontRight
    {
        ChangeState(TURN);
    }
    else if (regions[F] < d && regions[FL] < d && regions[FR] > d) //case 6: Front & FrontLeft
    {
        ChangeState(TURN);
    }
    else if (regions[F] < d && regions[FL] < d && regions[FR] < d) //case 7: Front & FrontLeft & FrontRight
    {
        ChangeState(TURN);
    }
    else if (regions[F] > d && regions[FL] < d && regions[FR] < d) //case 8: FrontLeft & FrontRight
    {
        ChangeState(FIND_WALL);
    }
    else
    {
        ROS_ERROR("Wrong State: This Shouldn't happen. Something's wrong.");
    }
}

double getYawFromQuaternion(geometry_msgs::Quaternion q){
    tf::Quaternion Q(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 m(Q);
    double r, p, y;
    m.getRPY(r, p, y);
    return y;
}

void Laser_Callback(sensor_msgs::LaserScan msg){
    laser = msg;
    for (int i = 0; i < laser.ranges.size(); i++)
    {
        if (laser.ranges[i] >= laser.range_max)
        {
            laser.ranges[i] = laser.range_max;
        }
    }
    std::vector<float>::iterator it = laser.ranges.begin();
    regions[R] = *std::min_element(it+270, it+305);
    regions[FR] = *std::min_element(it+306, it+341);
    regions[F] = std::min(*std::min_element(it+342, it+359), *std::min_element(it, it+17));
    regions[FL] = *std::min_element(it+18, it+53);
    regions[L] = *std::min_element(it+54, it+90);

    checkLaser = true;

    double err_yaw = atan2(goalPos.position.y-currentPos.position.y, goalPos.position.x-currentPos.position.x) - getYawFromQuaternion(currentPos.orientation);
    err_yaw = Normalize_Angle(err_yaw);
    if (err_yaw > 0)
    {
        _side = LEFT;
    }
    else
    {
        _side = RIGHT;
    }

    FindNewState();
}

geometry_msgs::Twist Find_Wall(){
    geometry_msgs::Twist speed;
    speed.linear.x = max_v;
    return speed;
}

geometry_msgs::Twist Turn(){
    geometry_msgs::Twist speed;
    speed.linear.x = 0.0;
    if (_side == LEFT)
    {
        speed.angular.z = 1.2;
    }
    else if (_side == RIGHT)
    {
        speed.angular.z = -1.2;
    }
    return speed;
}

geometry_msgs::Twist Follow_Wall(){
    geometry_msgs::Twist speed;
    speed.linear.x = max_v / 2.0;
    return speed;
}




int main(int argc, char** argv){
    ros::init(argc, argv, "Follow_Wall");
    ros::NodeHandle nh;

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pose_sub = nh.subscribe("/odom", 1, Pose_Callback);
    laser_sub = nh.subscribe("/scan", 1, Laser_Callback);
    srv = nh.advertiseService("/Follow_Wall_Switch", Follow_Wall_Switch_Callback);
    go_to_point_client = nh.serviceClient<navigation_pkg::ActivateGoal>("/Go_To_Point_Switch");

    while (!checkPos || !checkLaser)
    {
        ROS_INFO("Waiting for /odom and /scan topics.");
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
        case FIND_WALL:
            speed = Find_Wall();
            break;
        case TURN:
            speed = Turn();
            break;
        case FOLLOW_WALL:
            speed = Follow_Wall();
            break;
        default:
            ROS_ERROR("Unknown State.");
            break;
        }

        vel_pub.publish(speed);
        double err_yaw = atan2(goalPos.position.y-currentPos.position.y, goalPos.position.x-currentPos.position.x) - getYawFromQuaternion(currentPos.orientation);
        err_yaw = Normalize_Angle(err_yaw);
        if (err_yaw > 0)
        {
            _side = LEFT;
        }
        else
        {
            _side = RIGHT;
        }
        
        r.sleep();
    }
    
}
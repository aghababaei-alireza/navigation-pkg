#include <navigation_pkg/Bug0.h>

namespace navigation_pkg
{
    Bug0::Bug0() :
    max_allow_v(0.26), max_allow_w(1.82), dist_precision(1.0e-1)
    {
        local_plan_srv = nh.advertiseService("/Bug0_LocalPlanner_Service", &Bug0::Bug0_Callback, this);
        pose_sub = nh.subscribe("/odom", 1, &Bug0::Pose_Callback, this);
        laser_sub = nh.subscribe("/scan", 1, &Bug0::Laser_Callback, this);
        vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        go_to_point_client = nh.serviceClient<navigation_pkg::ActivateGoal>("/Go_To_Point_Switch");
        follow_wall_client = nh.serviceClient<navigation_pkg::ActivateGoal>("/Follow_Wall_Switch");

        go_to_point_client.waitForExistence();
        follow_wall_client.waitForExistence();

        while (!checkPose || !checkLaser)
        {
            ROS_INFO("Waiting for /odom and /scan topics.");
            ros::spinOnce();
            ros::Duration(1).sleep();
        }
        

        ROS_INFO("Bug0 initialized successfully.");
    }

    bool Bug0::Bug0_Callback(navigation_pkg::Pose::Request& req, navigation_pkg::Pose::Response& resp){
        ROS_INFO("Bug0 Callback.");
        for (int i = 0; i < req.pose.size(); i++)
        {
            goal = req.pose[i];
            if (!Bug0::Bug0Algorithm(currentPos, goal))
            {
                ROS_ERROR("Unable to reach the target.");
                resp.success = false;
                return false;
            }
            ROS_INFO("Reached the target %d", i);
        }
        resp.success = true;
        return true;
    }

    void Bug0::Pose_Callback(nav_msgs::Odometry msg){
        currentPos.position = msg.pose.pose.position;
        currentPos.orientation = msg.pose.pose.orientation;
        currentVel = msg.twist.twist;
        checkPose = true;
    }

    void Bug0::Laser_Callback(sensor_msgs::LaserScan msg){
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
    }

    double Bug0::getYawFromQuaternion(geometry_msgs::Quaternion q){
        tf::Quaternion Q(q.x, q.y, q.z, q.w);
        tf::Matrix3x3 m(Q);
        double r, p, y;
        m.getRPY(r, p, y);
        return y;
    }

    void Bug0::ChangeState(State state){
        _state = state;
        navigation_pkg::ActivateGoal msgGo, msgFollow;
        switch (_state)
        {
        case GoToPoint:
            msgGo.request.activate = true;
            msgGo.request.pose = goal;
            go_to_point_client.call(msgGo);
            msgFollow.request.activate = false;
            msgFollow.request.pose = goal;
            follow_wall_client.call(msgFollow);
            break;
        case FollowWall:
            msgGo.request.activate = false;
            msgGo.request.pose = goal;
            go_to_point_client.call(msgGo);
            msgFollow.request.activate = true;
            msgFollow.request.pose = goal;
            follow_wall_client.call(msgFollow);
            break;
        default:
            ROS_ERROR("Unknown State. This shouldn't happen. Something's wrong.");
            break;
        }
    }

    double Bug0::Normalize_Angle(double angle){
        double a;
        if (std::abs(angle) > M_PI)
        {
            a = angle - (2 * M_PI * angle) / std::abs(angle);
        }
        return a;
    }

    bool Bug0::Bug0Algorithm(geometry_msgs::Pose& currentPos, geometry_msgs::Pose& goalPos){
        ros::Rate r(20);
        ChangeState(GoToPoint);
        double d = 0.75;

        while (true)
        {
            ros::spinOnce();
            switch (_state)
            {
            case GoToPoint:{
                if (regions[F] > laser.range_min && regions[F] < d)
                {
                    ChangeState(FollowWall);
                }
            }
                break;
            case FollowWall:{
                double desired_yaw = atan2(goalPos.position.y-currentPos.position.y, goalPos.position.x-currentPos.position.x);
                double err_yaw = Normalize_Angle(desired_yaw - getYawFromQuaternion(currentPos.orientation));
                if (std::abs(err_yaw) < M_PI/6.0 &&
                    regions[F] > 2*d && regions[FR] > d && regions[FL] > d)
                {
                    ChangeState(GoToPoint);
                }
                if (err_yaw > 0 && std::abs(err_yaw) > M_PI/6.0 && std::abs(err_yaw) < M_PI/2.0 &&
                    regions[L] > 2*d && regions[FL] > d)
                {
                    ChangeState(GoToPoint);
                }
                if (err_yaw < 0 && std::abs(err_yaw) > M_PI/6.0 && std::abs(err_yaw) < M_PI/2.0 &&
                    regions[R] > 2*d && regions[FR] > d)
                {
                    ChangeState(GoToPoint);
                }
            }
                break;
            default:
                ROS_ERROR("Unknown State. This shouldn't happen. Something's wrong.");
                return false;
                break;
            }
            r.sleep();
            ros::spinOnce();
            if (sqrt(pow(currentPos.position.x-goalPos.position.x, 2) + pow(currentPos.position.y-goalPos.position.y, 2)) < dist_precision)
            {
                return true;
            }
            
        }
        
    }


}; // namespace navigation_pkg

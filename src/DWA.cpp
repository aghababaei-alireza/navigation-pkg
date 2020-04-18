#include <navigation_pkg/DWA.h>
#include <sstream>

namespace navigation_pkg
{
    DWA::DWA()
    :dt(0.2), max_allow_v(0.26), max_allow_w(1.82), dv_a(2.5), dw_a(3.2), dv_b(dv_a), dw_b(dw_a), v_samples(10), w_samples(20),
    alpha(0.1), beta(0.8), gamma(0.1),
    robotWidth(0.306), angle_samples(5), line_samples(6), sl(3.5),
    checkPose(false), checkLaser(false), nh(ros::NodeHandle()), isShutDown(false)
    {
        local_plan_srv = nh.advertiseService("/DWA_LocalPlanner_Service", &DWA::DWA_Callback, this);
        pose_sub = nh.subscribe("/odom", 1, &DWA::Pose_Callback, this);
        laser_sub = nh.subscribe("/scan", 1, &DWA::Laser_Callback, this);
        vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        shutDown_srv = nh.advertiseService("/ShutDownLocalPlanning", &DWA::ShutdownCallback, this);
        debug_srv = nh.advertiseService("/debugLocal", &DWA::Debug_Callback, this);
        ROS_INFO("DWA initialized successfully.");
    }

    void DWA::Pose_Callback(nav_msgs::Odometry msg){
        currentPos.position = msg.pose.pose.position;
        currentPos.orientation = msg.pose.pose.orientation;
        currentVel = msg.twist.twist;
        checkPose = true;
    }

    void DWA::Laser_Callback(sensor_msgs::LaserScan msg){
        laser = msg;
        for (int i = 0; i < laser.ranges.size(); i++)
        {
            if (laser.ranges[i] >= laser.range_max)
            {
                laser.ranges[i] = laser.range_max;
            }
        }
        checkLaser = true;
    }

    bool DWA::DWA_Callback(navigation_pkg::Pose::Request& req, navigation_pkg::Pose::Response& resp){
        ROS_INFO("DWA Callback.");
        for (int i = 0; i < req.pose.size(); i++)
        {
            if (isShutDown){
                DWA::SetSpeed(0.0, 0.0);
                vel_pub.publish(speed);
                break;
            }
            while (!checkPose || !checkLaser)
            {
                ROS_INFO("Wainting for topics: /odom and /scan.");
                ros::spinOnce();
                ros::Duration(0.5).sleep();
            }
            ROS_INFO("Subscribed to topics: /odom and /scan.");
            goal = req.pose[i];
            DWA::DynamicWindowApproach(currentPos, goal);
            checkPose = false;
            checkLaser = false;
            ROS_INFO("Reached i = %d", i);
        }
        isShutDown = false;
        return true;
    }

    void DWA::DynamicWindowApproach(geometry_msgs::Pose& currentPos, geometry_msgs::Pose& goalPos){
        
        while (true)
        {
            if (isShutDown){
                DWA::SetSpeed(0.0, 0.0);
                vel_pub.publish(speed);
                break;
            }
            // std::string c = "";
            ROS_INFO("////////////////////////////////////////");
            ROS_INFO("/// Dynamic Window Approach Started. ///");
            ROS_INFO("////////////////////////////////////////");
            double desiredV = DWA::CalcDesiredVel(currentPos, goalPos);
            // ROS_INFO("Desired v calculated = %f.", desiredV);
            DWA::GenerateDynamicWindow(currentVel.linear.x, currentVel.angular.z, min_window_v, max_window_v, min_window_w, max_window_w);
            // ROS_INFO("Dynamic Window generated: v[%.3f, %.3f] w[%.3f, %.3f]", min_window_v, max_window_v, min_window_w, max_window_w);
            v_increment = (max_window_v - min_window_v) / v_samples;
            w_increment = (max_window_w - min_window_w) / w_samples;

            double optimal_cost = 0.0;

            // ROS_INFO("v_inc = %.10f\tw_inc = %.10f", v_increment, w_increment);

            for (double v = min_window_v; v <= max_window_v; v += v_increment)
            {
                // c.append("\nv="+std::to_string(v)+"\t");
                for (double w = min_window_w; w <= max_window_w; w += w_increment)
                {
                    // ROS_INFO("****************************************");
                    // ROS_INFO("*** v = %.9f, w = %.9f ***", v, w);
                    // ROS_INFO("****************************************");
                    double dist = DWA::GetMinimumDistanceToObstacle(v, w);
                    // std::stringstream ss;
                    // ss << std::fixed << std::setprecision(12) << dist;
                    // c.append(std::to_string(dist)+"\t");
                    // if(v > sqrt(2 * dv_b * dist) || w > sqrt(2 * dw_b * dist)) continue; //Only take admissible velocities. Non-admissible velocities will be ignored
                    double heading = DWA::CalcTargetHeading(currentPos, goalPos, v, w);
                    double clearance = DWA::CalcClearance(dist, v, w);
                    double velocity = DWA::CalcVelocity(v, desiredV);
                    // ss << std::fixed << std::setprecision(4) << "(" << heading << ", " << clearance << ", " << velocity << ")\t";
                    // c.append(ss.str() + "\t");
                    double cost = (alpha * heading) + (beta * clearance) + (gamma * velocity);
                    // ROS_INFO("dist = %.9f, heading = %.9f, clearance = %.9f, velocity = %.9f, cost = %.9f", dist, heading, clearance, velocity, cost);
                    if (cost > optimal_cost)
                    {
                        optimal_cost = cost;
                        DWA::SetSpeed(v, w);
                    }
                }

            }
            vel_pub.publish(speed);
            // ROS_INFO("%s", c.c_str());
            ROS_INFO("Chosen velocity: v = %.10f, w = %.10f, optimal_cost = %.10f", speed.linear.x, speed.angular.z, optimal_cost);
            ros::Duration(dt).sleep();
            ros::spinOnce();
            if (DWA::GetDistance(currentPos, goalPos) < 1.0e-1)
            {
                DWA::SetSpeed(0.0,0.0);
                vel_pub.publish(speed);
                ROS_INFO("Reached the target.");
                break;
            }
        }
    }

    double DWA::CalcDesiredVel(geometry_msgs::Pose currentPos, geometry_msgs::Pose goalPos){
        double dist = DWA::GetDistance(currentPos, goalPos);
        double des_v = dist * 0.5;
        if (des_v > max_allow_v)
        {
            des_v = max_allow_v;
        }
        ROS_INFO("Calculating desired v: CurrentPos(%.3f, %.3f, %.3f) GoalPos(%.3f, %.3f) Distance(%.3f) DesiredV(%.9f)", currentPos.position.x, currentPos.position.y, DWA::getYawFromQuaternion(currentPos.orientation), goalPos.position.x, goalPos.position.y, dist, des_v);
        return des_v;
    }

    double DWA::GetDistance(geometry_msgs::Pose poseA, geometry_msgs::Pose poseB){
        return sqrt(pow(poseA.position.x - poseB.position.x, 2) + pow(poseA.position.y - poseB.position.y, 2) + pow(poseA.position.z - poseB.position.z, 2));
    }

    void DWA::GenerateDynamicWindow(double Vc, double Wc, double& Vmin, double& Vmax, double& Wmin, double& Wmax){
        Vmin = Vc - (dv_b * dt);
        if (Vmin < 0.0)
        {
            Vmin = 0.0;
        }
        Vmax = Vc + (dv_a * dt);
        if (Vmax > max_allow_v)
        {
            Vmax = max_allow_v;
        }
        Wmin = Wc - (dw_b * dt);
        if (Wmin < -max_allow_w)
        {
            Wmin = -max_allow_w;
        }
        Wmax = Wc + (dw_a * dt);
        if (Wmax > max_allow_w)
        {
            Wmax = max_allow_w;
        }
    }

    /*
    double DWA::GetMinimumDistanceToObstacle(double v, double w){
        geometry_msgs::Pose2D A;    //current position (start point of the arc)
        geometry_msgs::Pose2D B;    //End point of the arc
        geometry_msgs::Pose2D C;    //center point of the arc - C.theta = angle of the arc(angular distance between A and B)
        geometry_msgs::Pose2D D;    //Moving point on the arc (from A to B) - representing the point where the laser beam intersects with the arc
        geometry_msgs::Pose2D E;    //Moving point on the line representing the robot width - E.theta = angle of the laser beam which intersects to point E
        double rad = (v/w);         //Radius of the arc
        double a;                   //LaserScanner angle (relative to the vertical axis - counter clock wise = positive)
        double b;                   //angle changing from 0 to C.theta 
        int coeff = 50;             //Number of steps to plan

        A = DWA::SetPose2D(currentPos.position.x, currentPos.position.y, DWA::getYawFromQuaternion(currentPos.orientation));
        C = DWA::SetPose2D(A.x- rad * sin(A.theta), A.y + rad * cos(A.theta), w * dt * coeff);
        B = DWA::SetPose2D(C.x + rad * sin(A.theta + C.theta), C.y - rad * cos(A.theta + C.theta), A.theta + C.theta);
        angle_increment = C.theta / angle_samples;

        double min_dist = std::abs(rad * C.theta);
        bool found = false;

        for (b = 0; b <= C.theta; b+=angle_increment)
        {
            if (found)
            {
                break;
            }
            
            D = DWA::SetPose2D(C.x + rad * sin(A.theta + b), C.y - rad * cos(A.theta + b), A.theta + b);
            a = atan2(D.y - A.y, D.x - A.x) - (A.theta);
            line_increment = robotWidth / line_samples;

            for (double lE = -line_samples*line_increment/2.0; lE <= line_samples*line_increment/2.0; lE+=line_increment)
            {
                E = DWA::SetPose2D(D.x + lE*sin(A.theta+b), D.y - lE*cos(A.theta+b), std::abs(atan2(E.y-A.y, E.x-A.x)-(A.theta)));
                double l_AE = std::abs(rad * b); //Length of the arc AE (More precisely AD)
                double d_AE = sqrt(pow(E.x - A.x, 2) + pow(E.y - A.y, 2));
                double laser_AE = DWA::getLaserDist(E.theta);
                if (laser_AE <= d_AE)
                {
                    min_dist = l_AE;
                    found = true;
                    break;
                }
                
            }
            
        }
        return min_dist;
    }*/

    double DWA::CalcTargetHeading(geometry_msgs::Pose currentPos, geometry_msgs::Pose goalPos, double v, double w){
        geometry_msgs::Pose nextPos, center;
        double currentTheta = DWA::getYawFromQuaternion(currentPos.orientation);
        double nextTheta = currentTheta + (w * dt);
        double radius = (v / w);
        center.position.x = currentPos.position.x - (radius * sin(currentTheta));
        center.position.y = currentPos.position.y + (radius * cos(currentTheta));
        center.position.z = currentPos.position.z;

        nextPos.position.x = center.position.x + (radius * sin(currentTheta + w * dt));
        nextPos.position.y = center.position.y - (radius * cos(currentTheta + w * dt));
        nextPos.position.z = center.position.z;

        double targetAngle = atan2((goalPos.position.y - nextPos.position.y), (goalPos.position.x - nextPos.position.x));
        double headingAngle = std::abs(targetAngle - nextTheta);
        return (M_PI - headingAngle) / M_PI;
    }

    /*
    double DWA::CalcClearance(double distance, double v, double w){
        double T_b = std::max(v/dv_b, w/dw_b);  //Breakage time
        double T_col = distance / v;            //Collision time
        double T_max = sl / max_allow_v;  //Max admissible collision time

        if (T_col <= T_b)
        {
            return 0.0;
        }
        else if (T_col > T_b && T_col < T_max)
        {
            return (T_col - T_b)/(T_max - T_b);
        }
        else
        {
            return 1.0;
        }
    }*/

    double DWA::CalcVelocity(double v, double des_v){
        return v / des_v;
    }

    void DWA::SetSpeed(double Lx, double Az){
        speed.linear.x = Lx;
        speed.linear.y = 0.0;
        speed.linear.z = 0.0;
        speed.angular.x = 0.0;
        speed.angular.y = 0.0;
        speed.angular.z = Az;
    }

    double DWA::getYawFromQuaternion(geometry_msgs::Quaternion q){
        tf::Quaternion Q(q.x, q.y, q.z, q.w);
        tf::Matrix3x3 m(Q);
        double r, p, y;
        m.getRPY(r, p, y);
        return y;
    }

    geometry_msgs::Pose2D DWA::SetPose2D(double _x, double _y, double _theta){
        geometry_msgs::Pose2D p;
        p.x = _x;
        p.y = _y;
        p.theta = _theta;
        return p;
    }

    double DWA::getLaserDist(double angle){
        while (angle < laser.angle_min)
        {
            angle += (laser.angle_max - laser.angle_min);
        }
        while (angle > laser.angle_max)
        {
            angle -= (laser.angle_max - laser.angle_min);
        }
        
        
        double percent = (angle - laser.angle_min) / (laser.angle_max - laser.angle_min);
        int index = (int)round(percent * laser.ranges.size());
        return laser.ranges[index];
    }

    bool DWA::ShutdownCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp){
        isShutDown = true;
        return true;
    }

    
    double DWA::GetMinimumDistanceToObstacle(double v, double w){
        geometry_msgs::Pose2D A;            //current position (start point of the arc)
        // geometry_msgs::Pose2D B;            //End point of the arc
        // geometry_msgs::Pose2D C;            //center point of the arc - C.theta = angle of the arc(angular distance between A and B)
        // double rad = (v/w);                 //Radius of the arc

        A = DWA::SetPose2D(currentPos.position.x, currentPos.position.y, DWA::getYawFromQuaternion(currentPos.orientation));
        // C = DWA::SetPose2D(A.x - rad * sin(A.theta), A.y + rad * cos(A.theta), w * dt);
        // B = DWA::SetPose2D(C.x + rad * sin(A.theta + C.theta), C.y - rad * cos(A.theta + C.theta), A.theta + C.theta);

        //from now on, we assume that the robot is in the position of point A and its orientation has the value of A.theta + w*dt
        A.theta += w*dt;

        double front = 0.5;                 //distance to be clear in front of the robot
        double side = robotWidth/2.0 + 0.1; //distance to be clear on the left and right side of the robot 

        geometry_msgs::Pose2D F, FR, FL;    //Front, FrontRight, FrontLeft Point
        F = DWA::SetPose2D(A.x + front*cos(A.theta), A.y + front*sin(A.theta), 0.0);
        FR = DWA::SetPose2D(F.x + side*sin(A.theta), F.y - side*cos(A.theta), 0.0);
        FL = DWA::SetPose2D(F.x - side*sin(A.theta), F.y + side*cos(A.theta), 0.0);

        FR.theta = 2*M_PI - (A.theta - atan2(FR.y-A.y, FR.x-A.x));  //in laser frame
        FL.theta = atan2(FL.y-A.y, FL.x-A.x) - A.theta;             //in laser frame

        double dist = 0.0;
        double dist_max = 0.0;

        for (double a = 3*M_PI_2; a <= FR.theta; a += laser.angle_increment)
        {
            geometry_msgs::Pose2D P;
            P.x = (A.y - A.x*tan(a-3*M_PI_2+A.theta-M_PI_2) -FR.y + FR.x*tan(A.theta)) / (tan(A.theta) - tan(a-3*M_PI_2+A.theta-M_PI_2));
            P.y = FR.y + tan(A.theta)*(P.x - FR.x);
            double d = sqrt(pow(P.x-A.x, 2) + pow(P.y-A.y, 2));
            double l = DWA::getLaserDist(a + w*dt);
            if (l >= d)
            {
                l = d;
            }
            dist += l/d;
            dist_max += d/d;
            // ROS_INFO("P(%.3f, %.3f) d(%.10f) l(%.10f) dist(%.10f) dist_max(%.3f)", P.x, P.y, d, l, dist, dist_max);
        }
        for (double a = FR.theta; a <= 2*M_PI; a += laser.angle_increment)
        {
            geometry_msgs::Pose2D P;
            P.x = (F.y - F.x*tan(A.theta-M_PI_2) - A.y + A.x*tan(A.theta+a-2*M_PI)) / (tan(A.theta+a-2*M_PI) - tan(A.theta-M_PI_2));
            P.y = tan(A.theta-M_PI_2)*(P.x - F.x) + F.y;
            double d = sqrt(pow(P.x-A.x, 2) + pow(P.y-A.y, 2));
            double l = DWA::getLaserDist(a+ w*dt);
            if (l >= d)
            {
                l = d;
            }
            dist += l/d;
            dist_max += d/d;
            // ROS_INFO("P(%.3f, %.3f) d(%.10f) l(%.10f) dist(%.10f) dist_max(%.3f)", P.x, P.y, d, l, dist, dist_max);
        }
        for (double a = 0; a <= FL.theta; a += laser.angle_increment)
        {
            geometry_msgs::Pose2D P;
            P.x = (F.y - F.x*tan(A.theta-M_PI_2) - A.y + A.x*tan(A.theta+a)) / (tan(A.theta+a) - tan(A.theta-M_PI_2));
            P.y = tan(A.theta-M_PI_2)*(P.x-F.x) + F.y;
            double d = sqrt(pow(P.x-A.x, 2) + pow(P.y-A.y, 2));
            double l = DWA::getLaserDist(a + w*dt);
            if (l >= d)
            {
                l = d;
            }
            dist += l/d;
            dist_max += d/d;
            // ROS_INFO("P(%.3f, %.3f) d(%.10f) l(%.10f) dist(%.10f) dist_max(%.3f)", P.x, P.y, d, l, dist, dist_max);
        }
        for (double a = FL.theta; a <= M_PI; a += laser.angle_increment)
        {
            geometry_msgs::Pose2D P;
            P.x = (FL.y - FL.x*tan(A.theta) - A.y + A.x*tan(A.theta+a)) / (tan(A.theta+a) - tan(A.theta));
            P.y = tan(A.theta) * (P.x - FL.x) + FL.y;
            double d = sqrt(pow(P.x-A.x, 2) + pow(P.y-A.y, 2));
            double l = DWA::getLaserDist(a+ w*dt);
            if (l >= d)
            {
                l = d;
            }
            dist += l/d;
            dist_max += d/d;
            // ROS_INFO("P(%.3f, %.3f) d(%.10f) l(%.10f) dist(%.10f) dist_max(%.3f)", P.x, P.y, d, l, dist, dist_max);
        }

        return dist/dist_max;
    }

    double DWA::CalcClearance(double distance, double v, double w){
        return distance;
    }

    bool DWA::Debug_Callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp){
        DWA::GenerateDynamicWindow(currentVel.linear.x, currentVel.angular.z, min_window_v, max_window_v, min_window_w, max_window_w);
        v_increment = (max_window_v - min_window_v) / v_samples;
        w_increment = (max_window_w - min_window_w) / w_samples;
        std::stringstream ss;
        for (double v = min_window_v; v <= max_window_v; v += v_increment)
        {
            ss << "v(" << std::fixed << std::setprecision(8) << v << ":\t";
            for (double w = min_window_w; w <= max_window_w; w += w_increment)
            {
                char c;
                std::cin >> c;

                double dist = DWA::GetMinimumDistanceToObstacle(v, w);
                ss << std::fixed << std::setprecision(8) << dist << ", ";
            }
            ss << "\n";

        }
        ROS_INFO("%s", ss.str().c_str());
    }


}; // namespace navigation_pkg

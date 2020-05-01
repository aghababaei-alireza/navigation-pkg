#include <navigation_pkg/VFH.h>

namespace navigation_pkg
{
    VFH::VFH(){
        pose_sub = nh.subscribe("/odom", 1, &VFH::Pose_Callback, this);
        laser_sub = nh.subscribe("/scan", 1, &VFH::Laser_Callback, this);
        vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        local_plan_srv = nh.advertiseService("/VFH_LocalPlanner_Service", &VFH::VFH_Callback, this);
        shutDown_srv = nh.advertiseService("/ShutDownLocalPlanning", &VFH::ShutdownCallback, this);

        checkPose = false;
        checkLaser = false;
        isShutDown = false;

        ROS_INFO("VFH initialized successfully.");
    }

    void VFH::Pose_Callback(nav_msgs::Odometry msg){
        currentPos.position = msg.pose.pose.position;
        currentPos.orientation = msg.pose.pose.orientation;
        currentVel = msg.twist.twist;
        checkPose = true;
    }

    void VFH::Laser_Callback(sensor_msgs::LaserScan msg){
        laser = msg;
        for (int i = 0; i < laser.ranges.size(); i++)
        {
            if (laser.ranges[i] >= laser.range_max)
            {
                laser.ranges[i] = laser.range_max;
            }
        }
        certainty_grid.UpdateSensorValue(laser, VFH::getYawFromQuaternion(currentPos.orientation));
        checkLaser = true;
    }

    bool VFH::ShutdownCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp){
        isShutDown = true;
        return true;
    }

    bool VFH::VFH_Callback(navigation_pkg::Pose::Request& req, navigation_pkg::Pose::Response& resp){
        ROS_INFO("VFH_Callback.");
        while (!checkPose || !checkLaser)
        {
            ROS_INFO("Waiting for /odom and /scan topics.");
            ros::spinOnce();
            ros::Duration(1).sleep();
        }
        for (int i = 0; i < req.pose.size(); i++)
        {
            
            ROS_INFO("Subscribed to /odom and /scan topics.");

            if (isShutDown)
            {
                VFH::SetSpeed(0.0, 0.0);
                vel_pub.publish(speed);
                break;
            }
            
            goalPos = req.pose[i];
            VFH::VectorFieldHistogram();
            checkPose = false;
            checkLaser = false;
            ROS_INFO("Reached target i = %d", i);
        }
        if (isShutDown)
        {
            isShutDown = false;
            return false;
        }
        return true;
    }

    void VFH::SetSpeed(double Lx, double Az){
        speed.linear.x = Lx;
        speed.linear.y = 0.0;
        speed.linear.z = 0.0;
        speed.angular.x = 0.0;
        speed.angular.y = 0.0;
        speed.angular.z = Az;
    }

    double VFH::getYawFromQuaternion(geometry_msgs::Quaternion q){
        tf::Quaternion Q(q.x, q.y, q.z, q.w);
        tf::Matrix3x3 m(Q);
        double r, p, y;
        m.getRPY(r, p, y);
        return y;
    }

    double VFH::GetDistance(geometry_msgs::Pose poseA, geometry_msgs::Pose poseB){
        return sqrt(pow(poseA.position.x - poseB.position.x, 2) + pow(poseA.position.y - poseB.position.y, 2) + pow(poseA.position.z - poseB.position.z, 2));
    }

    void VFH::VectorFieldHistogram(){
        while (true)
        {
            if (isShutDown)
            {
                VFH::SetSpeed(0.0, 0.0);
                vel_pub.publish(speed);
                return;
            }
            
            ros::spinOnce();

            VFH::Polar_Histogram();
            VFH::Smoothed_Polar_Histogram();
            int k_c = VFH::Data_Reduction();
            VFH::Speed_Control(k_c);

            double delta_theta = DeltaRad(steer_direction, VFH::getYawFromQuaternion(currentPos.orientation));
            double Az = delta_theta * 2;
            Az = (std::abs(Az) > 1.82)? 1.82 * std::abs(Az)/Az: Az;
            VFH::SetSpeed(steer_magnitude, Az);
            vel_pub.publish(speed);

            ros::spinOnce();
            if (VFH::GetDistance(currentPos, goalPos) < 1.0e-1)
            {
                VFH::SetSpeed(0.0, 0.0);
                vel_pub.publish(speed);
                ROS_INFO("Reached target.");
                break;
            }
        }
    }

    void VFH::Polar_Histogram(){
        ROS_INFO("Entered Polar_Histogram.");
        double L_max = CELL_SIZE * (WINDOW_SIZE - 1) / 2;

        memset(POD, 0, 360 * sizeof(double));

        for (int yi = 0; yi < WINDOW_SIZE; yi++)
        {
            for (int xi = 0; xi < WINDOW_SIZE; xi++)
            {
                double c_ij = certainty_grid.cells[yi][xi];
                if (c_ij > 0)
                {
                    double dy = certainty_grid.CU2M(yi);
                    double dx = certainty_grid.CU2M(xi);

                    double d_ij = sqrt(dx*dx + dy*dy);
                    double beta_ij = atan2(dy, dx);

                    int index = H_ID(beta_ij * _RAD2DEG);

                    if (d_ij < L_max)
                    {
                        double a = 4.0;
                        double b = a / L_max;
                        double m_ij = c_ij*c_ij*(a - b * d_ij);
                        POD[index] = std::max(POD[index], m_ij);
                    }
                }
            }
        }
    }

    void VFH::Smoothed_Polar_Histogram(){
        ROS_INFO("Entered Smoothed_Polar_Histogram.");
        int l = 25;
        for (int i = 0; i < 360; i++)
        {
            smoothed_POD[i] = (l+1) * POD[i];
            int n = l + 1;

            for (int j = 1; j <= l; j++)
            {
                int i_p = H_ID(i - j);
                int i_m = H_ID(i + j);

                smoothed_POD[i] += (l - j + 1) * POD[i_p];
                smoothed_POD[i] += (l - j + 1) * POD[i_m];

                n += 2 * (l - j + 1);
            }
            smoothed_POD[i] /= n;
        }
        
    }

    int VFH::Data_Reduction(){
        ROS_INFO("Entered Data_Reduction.");
        #define S_MAX       (18*3)
        #define THRESHOLD   0.1

        target_direction = atan2(goalPos.position.y - currentPos.position.y, goalPos.position.x - currentPos.position.x);

        int min_i = -1;
        double min_value = 1000;

        for (int i = 0; i < 360; i++)
        {
            if (smoothed_POD[i] < THRESHOLD)
            {
                double dth = std::abs(DeltaRad(target_direction, i*_DEG2RAD));
                if (dth < min_value)
                {
                    min_value = dth;
                    min_i = i;
                }
            }
        }
        
        if (min_i < 0)
        {
            steer_magnitude = 0.0;
        }
        
        int k_n = min_i - 1;
        int k_f = min_i + 1;

        for (int count = 0; count < S_MAX; )
        {
            bool out = false;
            if (smoothed_POD[H_ID(k_n)] < THRESHOLD)
            {
                --k_n;
                ++count;
            }
            else
            {
                out = true;
            }
            
            if (smoothed_POD[H_ID(k_f)] < THRESHOLD)
            {
                ++k_f;
                ++count;
            }
            else
            {
                if (out)    break;
            }
        }
        int k_c = (k_n + k_f) / 2;
        return H_ID(k_c);
    }

    void VFH::Speed_Control(int k_c){
        ROS_INFO("Entered Speed_Control.");
        #define V_MAX           0.26
        #define V_MIN           0.01
        #define OMEGA_MAX       1.82

        steer_direction = k_c * _DEG2RAD;
        steer_magnitude = V_MAX * (1 - smoothed_POD[(int)(k_c)]/1.0);

        if (steer_magnitude < V_MIN) steer_magnitude = V_MIN;

        double Omega = DeltaRad(steer_direction, VFH::getYawFromQuaternion(currentPos.orientation));

        steer_magnitude = steer_magnitude * (1.0 - std::abs(Omega)/OMEGA_MAX);

        if(steer_magnitude < V_MIN) steer_magnitude = V_MIN;
    }


}; // namespace navigation_pkg

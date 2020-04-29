#include <navigation_pkg/VFH_LocalMap.h>
#include <cstring>

namespace navigation_pkg
{
    VFH_LocalMap::VFH_LocalMap(){
        VFH_LocalMap::Clear();
    }

    void VFH_LocalMap::Clear(){
        memset(cells, 0, sizeof(cells));
    }

    void VFH_LocalMap::DrawLine (int x1, int y1, int x2, int y2, double value){
        int deltax = abs(x2 - x1);        // The difference between the x's
        int deltay = abs(y2 - y1);        // The difference between the y's
        int x = x1;                       // Start x off at the first pixel
        int y = y1;                       // Start y off at the first pixel
        int xinc1, xinc2;
        int yinc1, yinc2;
        int den, num, numadd;
        int numpixels, curpixel;
        
        if (x2 >= x1)	xinc1 = 1,	xinc2 = 1;
        else			xinc1 = -1, xinc2 = -1;
        if (y2 >= y1) 	yinc1 = 1,	yinc2 = 1;
        else			yinc1 = -1,	yinc2 = -1;
        
        if (deltax >= deltay) {         // There is at least one x-value for every y-value
            xinc1 = 0;                  // Don't change the x when numerator >= denominator
            yinc2 = 0;                  // Don't change the y for every iteration
            den = deltax;
            num = deltax / 2;
            numadd = deltay;
            numpixels = deltax;         // There are more x-values than y-values
        }
        else {                         // There is at least one y-value for every x-value
            xinc2 = 0;                  // Don't change the x for every iteration
            yinc1 = 0;                  // Don't change the y when numerator >= denominator
            den = deltay;
            num = deltay / 2;
            numadd = deltax;
            numpixels = deltay;         // There are more y-values than x-values
        }
        
        for (curpixel = 0; curpixel <= numpixels; ++curpixel) {
            if (VFH_LocalMap::IsIn (x, y))
                cells[y][x] = value;
            else
                break;

            num += numadd;              // Increase the numerator by the top of the fraction
            if (num >= den) {            // Check if numerator >= denominator
                num -= den;               // Calculate the new numerator value
                x += xinc1;               // Change the x as appropriate
                y += yinc1;               // Change the y as appropriate
            }
            x += xinc2;                 // Change the x as appropriate
            y += yinc2;                 // Change the y as appropriate
        }
    }

    void VFH_LocalMap::UpdateSensorValue (sensor_msgs::LaserScan laser, double robot_theta){
        // ROS_INFO("Update Sensor Values.");
        VFH_LocalMap::Clear();
        double resolution = laser.angle_increment;
        int _sensor_scan_count = laser.ranges.size();

        for (int i = 0; i < _sensor_scan_count; ++i) {
            double v = laser.ranges[i];

            if (v > laser.range_min) {
                v *= 0.9;

                double theta = H_ID(robot_theta + i*resolution);
                double x = v*cos(theta);
                double y = v*sin(theta);
                
                int sx = INT_(M2CU (0.0));
                int sy = INT_(M2CU (0.0));
                int ex = INT_(M2CU (x));
                int ey = INT_(M2CU (y));

                DrawLine (sx, sy, ex, ey, 0);
            }
        }
        for (int i = 0; i < _sensor_scan_count; ++i) {
            double v = laser.ranges[i];

            if (v > laser.range_min) {
                double theta = H_ID(robot_theta + i*resolution);
                double x = v*cos(theta);
                double y = v*sin(theta);
                
                int ex = INT_(M2CU (x));
                int ey = INT_(M2CU (y));

                if (v < laser.range_max) {
                    SetPixel (ex, ey, 1);
                }
            }
        }
    }


}; // namespace navigation_pkg

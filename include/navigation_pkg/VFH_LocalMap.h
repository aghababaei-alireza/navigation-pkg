#ifndef VFH_LOCALMAP_H
#define VFH_LOCALMAP_H

#include <cmath>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>

#define CELL_SIZE       0.1     //m
#define WINDOW_SIZE     33

#define _RAD2DEG	(180./M_PI)
#define _DEG2RAD	(M_PI/180.)

namespace navigation_pkg
{
    class VFH_LocalMap{
    public:
        double cells[WINDOW_SIZE][WINDOW_SIZE];

        VFH_LocalMap();
        ~VFH_LocalMap() {}

        inline double CU (double p) {return (p + CELL_SIZE/2) / CELL_SIZE; }
        inline int M2CU (double x) {return (int)(WINDOW_SIZE/2 + x/CELL_SIZE); }
        inline double CU2M (int x) {return (x - WINDOW_SIZE/2)*CELL_SIZE; }

        inline bool IsIn (int x, int y) {return (0 <= x && x < WINDOW_SIZE) && (0 <= y && y < WINDOW_SIZE); }
        inline void SetPixel (int x, int y, double value) { if (IsIn(x, y)) cells[y][x] = value; }
        
        void DrawLine (int x1, int y1, int x2, int y2, double value);

        void Clear ();

        void UpdateSensorValue (sensor_msgs::LaserScan laser, double robot_theta);
    };
}; // namespace navigation_pkg

inline double DeltaRad (double ang1, double ang2)
{
	double da = ang1 - ang2;
	if (-M_PI < da && da < M_PI) return da;
	else {
		da = fmod (da, 2*M_PI);
		if (M_PI <= da) return da - 2*M_PI;
		else if (da <= -M_PI) return da + 2*M_PI;
		else return da;
	}
	return da;
}

inline int INT_ (const double a)
{
	// return (long)floor (a + 0.5);
	return (0 < a)? (int)(a + 0.5) : (int)(a - 0.5);
}

inline int H_ID (double theta)
{
	double alpha = 1.;
	return (int)(theta + 10*360 + alpha/2.)%360;
}

#endif
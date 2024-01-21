#include "Eigen/Eigen"
#include "pros/motors.hpp"
#include "pros/llemu.hpp"
#include "api.h"
#include <cmath>
#include <iostream>

#define M_PI		3.14159265358979323846
#define M_PI_2		1.57079632679489661923
#define M_PI_4		0.78539816339744830962
#define M_1_PI		0.31830988618379067154
#define M_2_PI		0.63661977236758134308

extern float radToDeg(float rad);
extern float degToRad(float deg);
extern float rollAngle180(float angle);
extern inline double boundAngleRadians(double angle);

extern inline double deltaInHeading(double targetHeading, double currentHeading);
extern float distance(double x1, double y1, double x2, double y2) ;


extern float pointAngleDifference(double x1, double y1, double x2, double y2);
//odom movements




    


extern float angleError(float angle1, float angle2, bool radians);
extern int sgn(float x);
extern float getCurvature(double posex, double posey, double posetheta, double otherx, double othery, double othertheta);
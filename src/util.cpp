#include "util.hpp"

float radToDeg(float rad) { return rad * 180 / M_PI; }
float degToRad(float deg) { return deg * M_PI / 180; }
float rollAngle180(float angle) {
    while (angle < -180) {
        angle += 360;
    }

    while (angle >= 180) {
        angle -= 360;
    }

    return angle;
}
inline double boundAngleRadians(double angle) {
    angle = fmod(angle, M_PI_2);
    if (angle < -M_PI) angle += 2*M_PI;
    if (angle > M_PI) angle -= 2*M_PI;
    return angle;
}

inline double deltaInHeading(double targetHeading, double currentHeading) {
  return boundAngleRadians(targetHeading - currentHeading);
}
float distance(double x1, double y1, double x2, double y2) 
{ 
    // Calculating distance 
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0); 
} 

float pointAngleDifference(double x1, double y1, double x2, double y2){
    return std::atan2(y2- y1, x2 - x1); 
}

float angleError(float angle1, float angle2, bool radians) {
    float max = radians ? 2 * M_PI : 360;
    float half = radians ? M_PI : 180;
    angle1 = fmod(angle1, max);
    angle2 = fmod(angle2, max);
    float error = angle1 - angle2;
    if (error > half) error -= max;
    else if (error < -half) error += max;
    return error;
}
int sgn(float x) {
    if (x < 0) return -1;
    else return 1;
}
float getCurvature(double posex, double posey, double posetheta, double otherx, double othery, double othertheta) {
    // calculate whether the pose is on the left or right side of the circle
    float side = sgn(std::sin(posetheta) * (otherx - othery) - std::cos(posetheta) * (othery - posey));
    // calculate center point and radius
    float a = -std::tan(posetheta);
    float c = std::tan(posetheta) * posex - posey;
    float x = std::fabs(a * otherx + othery + c) / std::sqrt((a * a) + 1);
    float d = std::hypot(otherx - posex, othery - posey);

    // return curvature
    return side * ((2 * x) / (d * d));
}
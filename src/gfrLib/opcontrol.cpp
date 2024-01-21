#include "gfrLib/opcontrol.hpp"

using namespace gfrLib;

float defaultDriveCurve(float input, float scale) {
    if (scale != 0) {
        return (powf(2.718, -(scale / 10)) + powf(2.718, (fabs(input) - 127) / 10) * (1 - powf(2.718, -(scale / 10)))) *
               input;
    }
    return input;
}

void Chassis::tank(float left, float right, float curve) {
    leftMotors->move(defaultDriveCurve(left, curve));
    rightMotors->move(defaultDriveCurve(right, curve));
}


void Chassis::arcade(float lateral, float angular, float curve) {
    double leftmotorsmove = lateral + angular;
    double rightmotorsmove = lateral - angular;
    leftMotors->move(defaultDriveCurve(leftmotorsmove,curve));
    rightMotors->move(defaultDriveCurve(rightmotorsmove, curve));
}
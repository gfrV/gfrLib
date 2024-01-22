#include "gfrLib/chassis.hpp"

using namespace gfrLib;

float defaultDriveCurve(float input, float scale) {
    if (scale != 0) {
        return (powf(2.718, -(scale / 10)) + powf(2.718, (fabs(input) - 127) / 10) * (1 - powf(2.718, -(scale / 10)))) *
               input;
    }
    return input;
}

/**
 * @brief moves the bot using tank fashion(left controls leftside of the base, right controls rightside of the base)
 *
 * @param left left power
 * @param right right power
 * @param curve 0<curve; curve driver control stick values
 */

void Chassis::tank(float left, float right, float curve) {
    leftMotors->move(defaultDriveCurve(left, curve));
    rightMotors->move(defaultDriveCurve(right, curve));
}

/**
 * @brief moves the bot using arcade fashion(lateral controls forward/backward movements while angular puts turn bias on
 * the powers)
 *
 * @param lateral left power
 * @param angular right power
 * @param curve 0<curve; curve driver control stick values
 */
void Chassis::arcade(float lateral, float angular, float curve) {
    double leftmotorsmove = lateral + angular;
    double rightmotorsmove = lateral - angular;
    leftMotors->move(defaultDriveCurve(leftmotorsmove, curve));
    rightMotors->move(defaultDriveCurve(rightmotorsmove, curve));
}
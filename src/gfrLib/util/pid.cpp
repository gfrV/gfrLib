#include "gfrLib/util/pid.hpp"

#include "pros/rtos.hpp"

#include <cmath>
#include <iostream>

/**
 * @brief sets up PID
 *
 * @param kp proportional value
 * @param ki integral value
 * @param kd derivative value
 * @param iMax max integral windup
 * @param settleError default settle error
 * @param settleTime default settle time
 * @param maxSettleError max settle error
 * @param maxSettleTime max settle time
 * @param maxTime how long the entire movement should take
 *
 */
PID::PID(float kp, float ki, float kd, const float iMax, const float settleError, const float settleTime,
         const float maxSettleError, const float maxSettleTime, const float maxTime)
    : kp(kp), ki(ki), kd(kd), iMax(iMax), settleError(settleError), settleTime(settleError),
      maxSettleError(maxSettleError), maxSettleTime(maxSettleTime), maxTime(maxTime) {
    reset();
}

/**
 * @return is the robot settled or not
 *
 */
bool PID::isSettled() {
    if (std::fabs(lastError) < settleError) {
        if (settleTimer == 0) { settleTimer = pros::millis(); }

        if (pros::millis() - settleTimer > settleTime) return true;
    }

    if (std::fabs(lastError) < maxSettleError) {
        if (maxSettleTimer == 0) { maxSettleTimer = pros::millis(); }

        if (pros::millis() - maxSettleTimer > maxSettleTime) return true;
    }

    if (maxTimer == 0) { maxTimer = pros::millis(); }

    if (pros::millis() - maxTimer > maxTime) return true;

    return false;
}

/**
 * @brief updates the PID values in timestep
 *
 * @param target target position for the PID
 * @param actual the position it is currently at
 */
float PID::update(float target, float actual) {
    float error = target - actual;

    float deltaError = error - lastError;
    lastError = error;

    errorSum += error;

    if (std::fabs(errorSum) > iMax) { errorSum = 0; }

    return error * kp + deltaError * kd + errorSum * ki;
}

/**
 * @brief calculates error in timestep
 *
 * @param error current error between the target and actual position
 */
float PID::updateerror(float error) {
    float deltaError = error - lastError;
    lastError = error;

    errorSum += error;

    if (std::fabs(errorSum) > iMax) { errorSum = 0; }

    return error * kp + deltaError * kd + errorSum * ki;
}

/**
 * @brief resets the PID.
 *
 */
void PID::reset() {
    maxTimer = 0;
    settleTimer = 0;
    maxSettleTimer = 0;
    lastError = 0;
    errorSum = 0;
}
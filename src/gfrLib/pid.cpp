#include "gfrLib/pid.hpp"

#include "pros/rtos.hpp"

#include <cmath>
#include <iostream>

PID::PID(float kp, float ki, float kd, const float iMax, const float settleError, const float settleTime, const float maxSettleError, const float maxSettleTime, const float maxTime)
    : kp(kp), ki(ki), kd(kd), iMax(iMax), settleError(settleError), settleTime(settleError), maxSettleError(maxSettleError), maxSettleTime(maxSettleTime), maxTime(maxTime) {
    reset();
}

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

float PID::update(float target, float actual) {
    float error = target - actual;

    float deltaError = error - lastError;
    lastError = error;

    errorSum += error;

    if (std::fabs(errorSum) > iMax) {
        errorSum = 0;
    }

    return error * kp + deltaError * kd + errorSum * ki;
}

float PID::updateerror(float error) {

    float deltaError = error - lastError;
    lastError = error;

    errorSum += error;

    if (std::fabs(errorSum) > iMax) {
        errorSum = 0;
    }

    return error * kp + deltaError * kd + errorSum * ki;
}

void PID::reset() {
    maxTimer = 0;
    settleTimer = 0;
    maxSettleTimer = 0;
    lastError = 0;
    errorSum = 0;
}
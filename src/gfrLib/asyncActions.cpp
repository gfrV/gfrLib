#include "gfrLib/chassis.hpp"

using namespace gfrLib;

/**
 * @brief Wait until the robot has traveled a certain distance along the path
 *
 * @note Units are in inches if current motion is moveTo or follow, degrees if using turnTo
 *
 * @param dist the distance the robot needs to travel before returning
 */
void Chassis::wait_until(float error) {
    // do while to give the thread time to start
    do pros::delay(10);
    while (distTravelled <= error && distTravelled != -1);
}

/**
 * @brief Dequeues this motion and permits queued task to run
 */
void Chassis::end_motion() {
    // move the "queue" forward 1
    this->motionRunning = this->motionQueued;
    this->motionQueued = false;

    // permit queued motion to run
    this->mutex.give();
}

/**
 * @brief Wait until the robot has completed the path
 *
 */
void Chassis::wait_until_done() {
    do pros::delay(10);
    while (distTravelled != -1);
}

/**
 * @brief Cancels the currently running motion.
 * If there is a queued motion, then that queued motion will run.
 */
void Chassis::cancel_motion() {
    this->motionRunning = false;
    pros::delay(10); // give time for motion to stop
}

/**
 * @brief Indicates that this motion is queued and blocks current task until this motion reaches front of queue
 */
void Chassis::request_motion_start() {
    if (this->is_in_motion()) this->motionQueued = true; // indicate a motion is queued
    else this->motionRunning = true; // indicate a motion is running

    // wait until this motion is at front of "queue"
    this->mutex.take(TIMEOUT_MAX);
}

/**
 * @brief Cancels all motions, even those that are queued.
 * After this, the chassis will not be in motion.
 */
void Chassis::cancel_all_motions() {
    this->motionRunning = false;
    this->motionQueued = false;
    pros::delay(10); // give time for motion to stop
}

/**
 * @return whether a motion is currently running
 */
bool Chassis::is_in_motion() const { return this->motionRunning; }
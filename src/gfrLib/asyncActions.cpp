#include "gfrLib/chassis.hpp"

using namespace gfrLib;

//wait until certain time error in movement
void Chassis::wait_until(float error) {
    // do while to give the thread time to start
    do pros::delay(10);
    while (distTravelled <= error && distTravelled != -1);
}

//end the movement after its done
void Chassis::end_motion() {
    // move the "queue" forward 1
    this->motionRunning = this->motionQueued;
    this->motionQueued = false;

    // permit queued motion to run
    this->mutex.give();
}

//wait until the robot is done with its movement
void Chassis::wait_until_done() {
    do pros::delay(10);
    while (distTravelled != -1);
}

//cancel the motion running
void Chassis::cancel_motion() {
    this->motionRunning = false;
    pros::delay(10); // give time for motion to stop
}

//request the start of motion
void Chassis::request_motion_start() {
    if (this->is_in_motion()) this->motionQueued = true; // indicate a motion is queued
    else this->motionRunning = true; // indicate a motion is running

    // wait until this motion is at front of "queue"
    this->mutex.take(TIMEOUT_MAX);

}

//cancel all motions in queue
void Chassis::cancel_all_motions() {
    this->motionRunning = false;
    this->motionQueued = false;
    pros::delay(10); // give time for motion to stop
}

//is the chassis in motion?
bool Chassis::is_in_motion() const { return this->motionRunning; }
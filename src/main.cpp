#include "main.h"

#include "globals.hpp"

//going to add odom soon
/**
 * MOVEMENT GUIDE-
 * move(distance, maxSpeed = 127); - move distance and settle
 * move_without_settle(distance, exitdistance, maxSpeed = 127); - how far the robot before reaching the targetted exit range and coast- reccomended: use this to chain to next movement, such as a turn, swing, or arc.
 * move_without_settletime(distance, timeout, maxSpeed = 127); - robot moves to a certain distance if allowed in specified time, coasts after.
 * movewithheading(distance, targetHeading, maxSpeed = 127 ) - robot travels a certain distance while approaching it with the indended target angle.
 * turn(angle, maxSpeed = 127) - turns to a specified angle
 * turnsmall(angle, maxSpeed = 127) - turns to a specified angle with low waittime
 * swing(angle, isLeft, maxSpeed = 127) - swings to a specified angle with (if isLeft = true, then swings with left, if false swings with right)
 * swing_without_settle(angle, isLeft, maxSped = 127) - swings to a specified angle with a side of the base however, coasts and doesn't stop the motors - recommnended: use this to chain to another move function afterwards
 * arc(angle, leftMult, rightMult, maxSpeed = 127) - creates a customizable arc to approach the target angle, leftMult and rightMult(0<=leftmult or rightmult<=1). However, if trying to do an arc with the wrong speeds on each side of the base, it might bug out(fix coming this week). 
 * arcnonsettle(angle, leftMult, rightMult, maxSpeed = 127) - creates a customizable arc to approach target, but doesnt settle after approaching the target- recommended: use this to chain to another movement afterwards
 * odometry movements getting published after odom is done.
 */


void initialize() {
    
    //auton selector- later
    pros::lcd::initialize();
    chassis.calibrate();
    
}


void disabled() {}


void competition_initialize() {}


void autonomous() {
    
    
}


void opcontrol() {
    
    
    while (true) {
        chassis.tank(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                     master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

        pros::lcd::print(1, "x: %f", chassis.x);
        pros::lcd::print(2, "y: %f", chassis.y);

        pros::delay(20);
    }
}
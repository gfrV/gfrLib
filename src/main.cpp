#include "main.h"

#include "globals.hpp"



void initialize() {
    
    //auton selector- later
    pros::lcd::initialize();
    chassis.calibrate();
    
}


void disabled() {}


void competition_initialize() {}


void autonomous() {
    //regular move
    chassis.move(24, 127);
    //async move
    chassis.move(24, 127, true);
    chassis.waitUntil(10);
    pros::lcd::print(1, "bot reached 10 inches!");
    chassis.waitUntilDone();
    //turn
    chassis.turn(90, 127);
    //async turn
    chassis.turn(90, 127, true);
    chassis.waitUntil(10); //error in degrees
    pros::lcd::print(1, "bot reached 10 degrees!");
    chassis.waitUntilDone();
    //swing
    chassis.swing(0, true);
    //async swing etc. etc.
    
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
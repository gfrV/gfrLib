#include "main.h"

#include "globals.hpp"



void initialize() {
    
    //auton selector- later
    pros::lcd::initialize();
    chassis.calibrate();
   
    
}


void disabled() {}


void competition_initialize() {}

gfrLib::gainScheduleParams drivegain{
    90,
    127,
    10,
    15
};

void autonomous(){
    chassis.move(24, drivegain, 127, false);
}

void opcontrol() {
    
    
    while (true) {
        chassis.tank(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                     master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y), 2.0);

        pros::lcd::print(1, "x: %f", chassis.x);
        pros::lcd::print(2, "y: %f", chassis.y);

        pros::delay(20);
    }
}
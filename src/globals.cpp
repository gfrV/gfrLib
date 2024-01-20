#include "globals.hpp"

// need to add lateral and angular PID settings for odometry movements, PID movements have different PIDS and so do odom movements for full customization.
//inertial sensor
pros::IMU inertial(12);

/*disregard this for now- only for 5327V*/
pros::Rotation leftrot(1);
pros::Rotation rightrot(2);
pros::Rotation middlerot(3);
//edit
const float wheelDiam = 2.75;
const float trackWidth = 13;
const float externalGearRatio = 1;

//left motors
pros::MotorGroup leftMotors(std::initializer_list<pros::Motor> {
    pros::Motor(-15, pros::E_MOTOR_GEAR_BLUE), // left front motor. port 8, reversed
    pros::Motor(-11, pros::E_MOTOR_GEAR_BLUE), // left middle motor. port 20, reversed
    pros::Motor(-14, pros::E_MOTOR_GEAR_BLUE),
});

//right motors
pros::MotorGroup rightMotors(std::initializer_list<pros::Motor> {
    pros::Motor(16, pros::E_MOTOR_GEAR_BLUE),
    pros::Motor(17, pros::E_MOTOR_GEAR_BLUE),
    pros::Motor(18, pros::E_MOTOR_GEAR_BLUE),
});


//PID Struct
//kP, kI, kD, iMax(integral max threshold-set to 0 as not neccesary), regular settle error, regular settle time for small error, max settle error, regular settle time for big error, how long the entire movement should take
Chassis chassis(
    //left motors, right motors, inertial sensor
    &leftMotors, &rightMotors, &inertial, 
    //wheel diameter, track width, gear ratio
    wheelDiam, trackWidth, externalGearRatio, 
    //forward PID - since many robots have altering weight from back to the front of the bot, and want to optimize
    PID(10, 0, 45, 0, 0.75, 250, 2, 750, 7000),
    //backward PID
    PID(10, 0, 45, 0, 0.75, 250, 1, 750, 7000),
    //turnPID - for bigger turns
    PID(3.5, 0, 15, 0, 1, 100, 3, 250, 1000),
    //smallturnPID - used for smaller turns to make exits faster
    PID(3.5, 0, 15, 0, 1, 100, 3, 250, 1000),
    //swing PID
    PID(3.5, 0, 15, 0, 1, 100, 3, 250, 1000),
    //arc PID
    PID(6, 0, 50, 0, 1, 100, 3, 250, 4000),
    //heading PID
    PID(5, 0, 15, 0, 1, 100, 3, 250, 1000)
);
//that's all it takes for integrated encoder odom!


pros::Controller master(pros::E_CONTROLLER_MASTER);

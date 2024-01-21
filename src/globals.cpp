#include "globals.hpp"
using namespace gfrLib;

/**
 * @brief wheel diameter
 *
 */
const float wheelDiam = 2.75;

/**
 * @brief trackWidth
 *
 * @note length from left to right side of your robot
 *
 */
const float trackWidth = 13;

/**
 * @brief externalGearRatio
 *
 * @note external input gear/output gear
 *
 */
const float externalGearRatio = 1;

/**
 * @brief inertial sensor
 *
 */
pros::IMU inertial(12);

/**
 * @brief sets up left motor group
 *
 *
 */
pros::MotorGroup leftMotors(std::initializer_list<pros::Motor> {
    pros::Motor(-15, pros::E_MOTOR_GEAR_BLUE), // left front motor. port 8, reversed
    pros::Motor(-11, pros::E_MOTOR_GEAR_BLUE), // left middle motor. port 20, reversed
    pros::Motor(-14, pros::E_MOTOR_GEAR_BLUE),
});

/**
 * @brief sets up right motor group
 *
 *
 */
pros::MotorGroup rightMotors(std::initializer_list<pros::Motor> {
    pros::Motor(16, pros::E_MOTOR_GEAR_BLUE),
    pros::Motor(17, pros::E_MOTOR_GEAR_BLUE),
    pros::Motor(18, pros::E_MOTOR_GEAR_BLUE),
});

/**
 * @brief Drivetrain struct
 *
 * Set up sensors to move around your bot!
 *
 * @param leftMotors left motors of the base
 * @param rightMotors right motors of the base
 * @param inertial imu
 * @param wheelDiameter the diameter of your wheel. sizes come in(2.75, 3.25, 4(new wheels), 4.125(old wheels))
 * @param gearRatio external gear ratio; ie. input gear/output gear
 *
 */
driveTrain drivetrain {&leftMotors, &rightMotors, &inertial, wheelDiam, trackWidth, externalGearRatio};

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
PID forwardPID = PID(10, 0, 45, 0, 0.75, 250, 2, 750, 7000); // pid for forward movement
PID backwardPID = PID(10, 0, 45, 0, 0.75, 250, 2, 750, 7000); // pid for backward movement
PID turnPID = PID(10, 0, 45, 0, 0.75, 250, 2, 750, 7000); // pid for turns
PID smallTurnPID = PID(10, 0, 45, 0, 0.75, 250, 2, 750,7000); // pid for small turns- to make exit conditions shorter and more customizable
PID swingPID = PID(10, 0, 45, 0, 0.75, 250, 2, 750, 7000); // pid for swing turns
PID arcPID = PID(10, 0, 45, 0, 0.75, 250, 2, 750, 7000); // pid for arc turns
PID headingPID = PID(10, 0, 45, 0, 0.75, 250, 2, 750, 7000); // pid to maintain a certain heading during movement

/**
 * @brief Struct for PID setup
 *
 * @param drivePID PID used for lateral movements
 * @param turnPID PID used for turn movements
 * @param smallTurnPID PID used for smaller turn movements; you can set this to the same as the turnPID if you want,
 * this is used to fine tune small turns.
 * @param swingPID PID used for swing movements
 * @param arcPID PID used for arc movements
 * @param headingPID PID used to keep the robot at a certain heading during movements
 *
 */
pidSetup myPids {forwardPID, backwardPID, turnPID, smallTurnPID, swingPID, arcPID, headingPID};

/**
 * @brief chassis setup
 *
 *
 */
Chassis chassis(drivetrain, myPids);
pros::Controller master(pros::E_CONTROLLER_MASTER);

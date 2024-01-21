#include "gfrLib/chassis.hpp"
#include "Eigen/Eigen"
#include "pros/motors.hpp"
#include "pros/llemu.hpp"
#include "api.h"
#include <cmath>
#include <iostream>

#define M_PI 3.14159265358979323846
#define M_PI_2 1.57079632679489661923
#define M_PI_4 0.78539816339744830962
#define M_1_PI 0.31830988618379067154
#define M_2_PI 0.63661977236758134308

using namespace gfrLib;

/**
 * @brief Struct for PID setup
 *
 * @param drivePID PID used for lateral movements
 * @param turnPID PID used for angular movements
 * @param smallTurnPID PID used for smaller turn movements; you can set this to the same as the turnPID if you want,
 * this is used to fine tune small turns.
 * @param swingPID PID used for swing movements
 * @param arcPID PID used for arc movements
 * @param headingPID PID used to keep the robot at a certain heading during movements
 *
 */

pidSetup::pidSetup(PID drivePID, PID backwardPID, PID turnPID, PID smallturnPID, PID swingPID, PID arcPID,
                   PID headingPID)
    : drivePID(drivePID), backwardPID(backwardPID), turnPID(turnPID), smallturnPID(smallturnPID), swingPID(swingPID),
      arcPID(arcPID), headingPID(headingPID) {}

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

driveTrain::driveTrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, pros::IMU* inertial,
                       const float wheelDiameter, const float trackWidth, const float gearRatio, const float defaultChasePower)
    : leftMotors(leftMotors), 
      rightMotors(rightMotors), 
      imu(inertial), 
      wheelDiameter(wheelDiameter),
      trackWidth(trackWidth), 
      gearRatio(gearRatio),
      defaultChasePower(defaultChasePower) {}

/**
 * @brief Struct for chassis
 *
 * Set up sensors to move around your bot!
 *
 * @param leftMotors left motors of the base
 * @param rightMotors right motors of the base
 * @param inertial imu
 * @param wheelDiameter the diameter of your wheel. sizes come in(2.75, 3.25, 4(new wheels), 4.125(old wheels))
 * @param gearRatio external gear ratio; ie. input gear/output gear
 * @param drivePID PID used for lateral movements
 * @param turnPID PID used for turn movements
 * @param smallTurnPID PID used for smaller turn movements; you can set this to the same as the turnPID if you want,
 * this is used to fine tune small turns.
 * @param swingPID PID used for swing movements
 * @param arcPID PID used for arc movements
 * @param headingPID PID used to keep the robot at a certain heading during movements
 *
 */
Chassis::Chassis(driveTrain drivetrain, pidSetup pidsetup)
    : leftMotors(drivetrain.leftMotors), 
      rightMotors(drivetrain.rightMotors), 
      imu(drivetrain.imu),
      wheelDiameter(drivetrain.wheelDiameter), 
      trackWidth(drivetrain.trackWidth), 
      gearRatio(drivetrain.gearRatio),
      drivePID(pidsetup.drivePID), 
      backwardPID(pidsetup.backwardPID), 
      turnPID(pidsetup.turnPID),
      smallturnPID(pidsetup.smallturnPID), 
      swingPID(pidsetup.swingPID), 
      arcPID(pidsetup.arcPID),
      headingPID(pidsetup.headingPID),
      defaultChasePower(drivetrain.defaultChasePower) {
    this->leftMotors->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    this->rightMotors->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
}

Pose odomPose(0, 0, 0);

/**
 * @brief calibrate chassis and set up odometry
 *
 */
void Chassis::calibrate() {
    imu->reset();
    leftMotors->tare_position();
    rightMotors->tare_position();

    pros::Task([=] {
        double prevLeft = 0;
        double prevRight = 0;
        double prevTheta = 0;
        x = 0;
        y = 0;

        while (true) {
            double left = leftMotors->get_positions()[0] * wheelDiameter * M_PI * gearRatio;
            double right = rightMotors->get_positions()[0] * wheelDiameter * M_PI * gearRatio;
            ;
            double dist = ((left - prevLeft) + (right - prevRight)) / 2;
            double thetaChange = rollAngle180(degToRad(imu->get_heading() - prevTheta));
            x += dist * sin(thetaChange);
            y += dist * cos(thetaChange);
            prevLeft = left;
            prevRight = right;
            prevTheta = imu->get_heading();
            odomPose.x = x;
            odomPose.y = y;
            odomPose.theta = degToRad(imu->get_heading());
        }
    });
}

/**
 * @brief set the heading of the bot specifically- pain to always change points when you just have to change heading.
 *
 * @param heading set heading in degrees
 */
void Chassis::set_heading(float heading) { imu->set_heading(heading); }

/**
 * @brief sets the odometry pose of the bot
 *
 * @param x1 set the x value
 * @param y1 set the y value
 * @param theta1 set the theta value(in degrees)
 *
 */
void Chassis::set_pose(float x1, float y1, float theta1) {
    x = x1;
    y = y1;
    imu->set_heading(theta1);
}

/**
 * @brief moves the bot forward using forwards or backwards PID
 *
 * @param distance distance the bot should move to(in inches)
 * @param maxSpeed the max speed the robot should travel in(out of 127).
 * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
 *
 */
void Chassis::move(float distance, float maxSpeed, bool async, float heading) {
    // dummy PID
    PID drivingPID = PID(0, 0, 0, 0, 0, 0, 0, 0, 0);
    float angle = 0;

    this->request_motion_start();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { move(distance, maxSpeed, false); });
        this->end_motion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    if (distance < 0) {
        PID drivingPID = backwardPID;
    } else {
        PID drivingPID = drivePID;
    }
    drivingPID.reset();
    float beginningLeft = leftMotors->get_positions()[0];
    float beginningRight = rightMotors->get_positions()[0];

    //check if heading is null or assign angle target
    if(heading != NULL){
        angle = heading;
    } else{
        angle = imu->get_heading();
    }
    do {
        // get current encoder positions
        float deltaLeft = leftMotors->get_positions()[0] - beginningLeft;
        float deltaRight = rightMotors->get_positions()[0] - beginningRight;
        // calculate distance travelled
        float distanceTravelled = (deltaLeft + deltaRight) / 2 * wheelDiameter * M_PI * gearRatio;
        distTravelled = distanceTravelled;
        // calculate error
        float error = rollAngle180(angle - imu->get_heading());
        // calculate pid outputs
        float pidAngOutput = headingPID.update(0, -error);
        float pidOutputLateral = drivingPID.update(distance, distanceTravelled);
        // cap max speeds
        pidOutputLateral = std::clamp(pidOutputLateral, -maxSpeed, maxSpeed);
        // run
        arcade(pidOutputLateral, pidAngOutput);
        pros::delay(20);

        // run while pid is not settled
    } while (!drivingPID.isSettled());
    // stop motors
    arcade(0, 0);
    // set distance travelled to -1 to let motion queue know
    distTravelled = -1;
    this->end_motion();
}

/**
 * @brief moves the bot forwards or backwards using the forward or backward PID and exits into the user specified next
 * movement. ANOTHER MOVEMENT AFTER THIS ONE IS REQUIRED.
 *
 * @param distance distance the bot should move to(in inches)
 * @param exitrange how much distance the robot should settle in(ie. want to move 24 inches: distance = 26, exitrange =
 * 2)
 * @param maxSpeed the max speed the robot can travel in(out of 127)
 * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
 *
 */
void Chassis::move_without_settle(float distance, float exitrange, float maxSpeed, bool async, float heading) {
    // dummy PID
    PID drivingPID = PID(0, 0, 0, 0, 0, 0, 0, 0, 0);
    float angle = 0;

    this->request_motion_start();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { move(distance, maxSpeed, false); });
        this->end_motion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    if (distance < 0) {
        PID drivingPID = backwardPID;
    } else {
        PID drivingPID = drivePID;
    }
    drivingPID.reset();
    float beginningLeft = leftMotors->get_positions()[0];
    float beginningRight = rightMotors->get_positions()[0];
    //check if heading is null or assign angle target
    if(heading != NULL){
        angle = heading;
    } else{
        angle = imu->get_heading();
    }
    float error = 0; // globalizing error for exit range calculations
    do {
        // get current encoder positions
        float deltaLeft = leftMotors->get_positions()[0] - beginningLeft;
        float deltaRight = rightMotors->get_positions()[0] - beginningRight;
        // calculate distance travelled
        float distanceTravelled = (deltaLeft + deltaRight) / 2 * wheelDiameter * M_PI * gearRatio;
        distTravelled = distanceTravelled;
        // calculate error
        error = rollAngle180(angle - imu->get_heading());
        // calculate pid outputs
        float pidAngOutput = headingPID.update(0, -error);
        float pidOutputLateral = drivingPID.update(distance, distanceTravelled);
        // cap max speeds
        pidOutputLateral = std::clamp(pidOutputLateral, -maxSpeed, maxSpeed);
        // run
        arcade(pidOutputLateral, pidAngOutput);
        pros::delay(20);

        // run while pid is not settled
    } while (!drivingPID.isSettled() || fabs(error) > exitrange);
    // dont stop motors, just coast them
    leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    // set distance travelled to -1 to let motion queue know
    distTravelled = -1;
    this->end_motion();
}

/**
 * @brief moves the bot forwards or backwards using the forward or backward PID and exits after a certain time is
 * reached
 *
 * @param distance distance the bot should move to(in inches)
 * @param timeout how much time the robot should move until
 * @param maxSpeed the max speed the robot can travel in(out of 127)
 * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
 *
 */
void Chassis::move_without_settletime(float distance, float timeout, float maxSpeed, bool async, float heading) {
    // dummy PID
    PID selectedPID = PID(0, 0, 0, 0, 0, 0, 0, 0, 0);
    float angle = 0;
    this->request_motion_start();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { move(distance, maxSpeed, false); });
        this->end_motion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    if (distance < 0) {
        PID drivingPID = backwardPID;
    } else {
        PID drivingPID = drivePID;
    }
    selectedPID.reset();
    // get prev values
    float beginningLeft = leftMotors->get_positions()[0];
    float beginningRight = rightMotors->get_positions()[0];
    
    //check if heading is null or assign angle target
    if(heading != NULL){
        angle = heading;
    } else{
        angle = imu->get_heading();
    }

    // timer start
    auto start = pros::millis();
    do {
        // get current encoder positions
        float deltaLeft = leftMotors->get_positions()[0] - beginningLeft;
        float deltaRight = rightMotors->get_positions()[0] - beginningRight;
        // calculate distance travelled
        float distanceTravelled = (deltaLeft + deltaRight) / 2 * wheelDiameter * M_PI * gearRatio;
        distTravelled = distanceTravelled;
        // calculate error
        float error = rollAngle180(angle - imu->get_heading());
        // calculate pid outputs
        float pidAngOutput = headingPID.update(0, -error);
        float pidOutputLateral = selectedPID.update(distance, distanceTravelled);
        // cap max speeds
        pidOutputLateral = std::clamp(pidOutputLateral, -maxSpeed, maxSpeed);
        // run
        arcade(pidOutputLateral, pidAngOutput);
        pros::delay(20);

        // run while pid is not settled
    } while (!selectedPID.isSettled() || (pros::millis() - start) < timeout);
    // dont stop motors, just coast them
    leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    // set distance travelled to -1 to let motion queue know
    distTravelled = -1;
    this->end_motion();
}

/**
 * @brief moves the bot to a certain point on the field
 *
 * @param x1 target x coordinate in inches
 * @param y1 target y coordinate in inches
 * @param timeout target y coordinate in inches
 * @param maxSpeed the max speed the robot should travel in(out of 127).
 * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
 *
 */
void Chassis::move_to_point(float x1, float y1, int timeout, float maxSpeed, bool async) {
    turnPID.reset();
    drivePID.reset();

    float prevLateralPower = 0;
    float prevAngularPower = 0;
    bool close = false;
    uint32_t start = pros::millis();
    this->request_motion_start();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { move_to_point(x1, y1, timeout, maxSpeed, false); });
        this->end_motion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    while (((start < timeout) || (!drivePID.isSettled() && !turnPID.isSettled()))) {
        heading = std::fmod(heading, 360);
        distTravelled += 10;
        // update error
        float deltaX = x1 - x;
        float deltaY = y1 - y;
        float targetTheta = fmod(radToDeg(M_PI_2 - atan2(deltaY, deltaX)), 360);
        float hypot = std::hypot(deltaX, deltaY);
        float diffTheta1 = angleError(heading, targetTheta, false);
        float diffTheta2 = angleError(heading, targetTheta + 180, false);
        float angularError = (std::fabs(diffTheta1) < std::fabs(diffTheta2)) ? diffTheta1 : diffTheta2;
        float lateralError = hypot * cos(degToRad(std::fabs(diffTheta1)));
        float lateralPower = drivePID.update(lateralError, 0);
        float angularPower = -turnPID.update(angularError, 0);

        if (distance(x1, y1, x, y) < 7.5) {
            close = true;
            maxSpeed = (std::fabs(prevLateralPower) < 30) ? 30 : std::fabs(prevLateralPower);
        }
        if (lateralPower > maxSpeed) lateralPower = maxSpeed;
        else if (lateralPower < -maxSpeed) lateralPower = -maxSpeed;
        if (close) angularPower = 0;

        prevLateralPower = lateralPower;
        prevAngularPower = angularPower;

        float leftPower = lateralPower + angularPower;
        float rightPower = lateralPower - angularPower;

        float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / maxSpeed;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }
        tank(leftPower, rightPower);
        pros::delay(10);
    }
    tank(0, 0);
    distTravelled = -1;
    this->end_motion();
}

/**
 * @brief moves the bot to a target pose(x,y,theta)
 *
 * @param x1 target x coordinate in inches
 * @param y1 target y coordinate in inches
 * @param theta1 target angle
 * @param timeout target y coordinate in inches
 * @param forwards movement is forwards or backwards
 * @param maxSpeed the max speed the robot should travel in(out of 127).
 * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
 * @param chasePower how fast the robot should go during the movement. If its higher, the accuracy of the movement is
 * less.
 * @param lead 0 < lead < 1 how much the movement should curve to the point
 * @param smoothness 0 < smoothness < 1 how smooth the movement should be.
 * @param gLead weight for ghost point
 *
 */
void Chassis::move_to_pose(float x1, float y1, float theta1, float timeout, MoveToPoseParams params, bool async) {
    this->request_motion_start();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() {
            move_to_pose(x1, y1, theta1, timeout, params, false);
        });
        this->end_motion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    // reset pids
    turnPID.reset();
    drivePID.reset();

    // calculate target theta in radians
    double targetTheta = M_PI_2 - degToRad(theta1);

    // prev powers
    float prevLateralPower = 0;
    float prevAngularPower = 0;

    // last pose
    float lastposex = x;
    float lastposey = y;
    float lastposetheta = heading;

    if (!params.forwards) targetTheta = fmod(targetTheta + M_PI, 2 * M_PI); // backwards movement

    bool close = false;
    if (params.chasePower == 0) params.chasePower = defaultChasePower; // make chasePower globalized in chassis setup
    // initial carrot
    double inCarrotX = (x1 - (cos(targetTheta) * params.lead * distance(x1, y1, x, y)));
    double inCarrotY = (y1 - (sin(targetTheta) * params.lead * distance(x1, y1, x, y)));
    // if gLead is 0, set glead to 1-dlead
    if (params.gLead == 0) { params.gLead = 1 - params.lead; }
    // timer
    auto start = pros::millis();
    // loop
    while ((!drivePID.isSettled() || pros::millis() - start < timeout)) {
        // rename curr odom vals
        double currX = x;
        double currY = y;
        double currHeading = heading; // for reference
        double currTheta = degToRad(heading);

        // if not forwards, add PI(180 deg) so it travels with the side of the bot the other way
        if (!params.forwards) currTheta += M_PI;
        // update distTravelled for async purposes
        distTravelled += distance(lastposex, lastposey, currX, currY);

        // update prev
        lastposex = currX;
        lastposey = currY;
        lastposetheta = currHeading;

        // check if close is true
        if (distance(x1, y1, x, y) < 7.5) { close = true; }

        // carrot - 2 times for each part
        double carrotX = (inCarrotX + (carrotX - inCarrotX) * (1 - params.gLead));
        double carrotY = (inCarrotY + (carrotY - inCarrotY) * (1 - params.gLead));

        if (close) { // settle behavior
            x1 = carrotX;
            y1 = carrotY;
        }

        // calculate error
        float angularError =
            angleError(pointAngleDifference(carrotX, carrotY, currX, currY), currTheta, true); // angular error
        float linearError = distance(carrotX, carrotY, currX, currY) * cos(angularError); // linear error

        // settling behabior
        if (close) angularError = angleError(targetTheta, currTheta, true); // settling behavior
        if (!params.forwards) linearError = -linearError;

        // get PID outputs
        float angularPower = -turnPID.update(radToDeg(angularError), 0);
        float linearPower = drivePID.update(linearError, 0);

        float curvature = fabs(getCurvature(currX, currY, currTheta, carrotX, carrotY, 0));
        if (curvature == 0) curvature = -1;
        float radius = 1 / curvature;

        // calculate the maximum speed at which the robot can turn
        // using the formula v = sqrt( u * r * g )
        if (radius != -1) {
            float maxTurnSpeed = sqrt(params.chasePower * radius * 9.8);
            // the new linear power is the minimum of the linear power and the max turn speed
            if (linearPower > maxTurnSpeed && !close) linearPower = maxTurnSpeed;
            else if (linearPower < -maxTurnSpeed && !close) linearPower = -maxTurnSpeed;
        }
        // prioritize turning over moving
        float overturn = fabs(angularPower) + fabs(linearPower) - params.maxSpeed - params.maxSpeed * params.smoothness;

        if (overturn > 0) linearPower -= linearPower > 0 ? overturn : -overturn;

        // calculate motor powers
        float leftPower = linearPower + angularPower;
        float rightPower = linearPower - angularPower;

        tank(leftPower, rightPower);
        pros::delay(10);
    }

    tank(0, 0);
    distTravelled = -1;
    this->end_motion();
}

/**
 * @brief turns the robot to a target angle
 *
 * @param heading target angle
 * @param isSmallTurn is the angle relatively small?
 * @param maxSpeed max speed the robot should move at
 * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
 *
 */
void Chassis::turn(float heading, bool isSmallTurn, float maxSpeed, bool async) {
    // dummy PID
    PID selectedPID = PID(0, 0, 0, 0, 0, 0, 0, 0, 0);
    if (isSmallTurn) {
        PID selectedPID = smallturnPID;
    } else {
        PID selectedPID = turnPID;
    }
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { turn(heading, maxSpeed, false); });
        this->end_motion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    selectedPID.reset();

    do {
        // calculate error
        float error = rollAngle180(heading - imu->get_heading());
        // update distTravelled for the time being for async actions
        distTravelled = error;
        // update pidOutput
        float pidOutput = selectedPID.update(0, -error);
        // cap max speed
        pidOutput = std::clamp(pidOutput, -maxSpeed, maxSpeed);
        // run
        arcade(0, pidOutput);
        pros::delay(20);
    } while (!selectedPID.isSettled());

    arcade(0, 0);
    distTravelled = -1;
    this->end_motion();
}

/**
 * @brief turns to a certain point(x,y)
 *
 * @param x1 the x coordinate in inches
 * @param y1 the y coordinate in inches
 * @param timeout how long the entire movement should take
 * @param forwards robot moving forwards or backwards during movement
 * @param maxSpeed how fast the robot should move during the movement
 * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
 *
 */
void Chassis::turn_to_point(float x1, float y1, int timeout, bool forwards, float maxSpeed, bool async) {
    this->request_motion_start();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { turn_to_point(x1, y1, timeout, forwards, maxSpeed, false); });
        this->end_motion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    float targetTheta;
    float deltaX, deltaY, deltaTheta;
    float motorPower;
    float startTheta = heading;
    std::uint8_t compState = pros::competition::get_status();
    distTravelled = 0;
    turnPID.reset();
    auto start = pros::millis();
    // main loop
    while ((pros::millis() - start < timeout) && !turnPID.isSettled() && this->motionRunning) {
        // update variables
        Pose pose = odomPose;
        pose.theta = (forwards) ? fmod(pose.theta, 360) : fmod(pose.theta - 180, 360);
        distTravelled += pose.distance(Pose(x1, y1));
        // update completion vars
        distTravelled = fabs(angleError(pose.theta, startTheta, false));

        deltaX = x - pose.x;
        deltaY = y - pose.y;
        targetTheta = fmod(radToDeg(M_PI_2 - atan2(deltaY, deltaX)), 360);

        // calculate deltaTheta
        deltaTheta = angleError(targetTheta, pose.theta, false);

        // calculate the speed
        motorPower = turnPID.update(0, -deltaTheta);

        // cap the speed
        motorPower = std::clamp(motorPower, -maxSpeed, maxSpeed);

        // move the drivetrain
        leftMotors->move(motorPower);
        rightMotors->move(-motorPower);

        pros::delay(10);
    }

    // stop the drivetrain
    leftMotors->move(0);
    rightMotors->move(0);
    // set distTraveled to -1 to indicate that the function has finished
    distTravelled = -1;
    this->end_motion();
}

/**
 * @brief swings to a target angle with specific side of base
 *
 * @param heading target angle
 * @param isLeft true = moves to the target angle left, false = moves to the target angle right
 * @param maxSpeed max speed the robot should move at
 * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
 *
 */
void Chassis::swing(float heading, bool isLeft, float maxSpeed, bool async) {
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { swing(heading, isLeft, maxSpeed, false); });
        this->end_motion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    swingPID.reset();

    do {
        // calculate error
        float error = rollAngle180(heading - imu->get_heading());
        // set distTravelled to error for async purposes
        distTravelled = error;
        // calculate pidOutput
        float pidOutput = swingPID.update(0, -error);
        // hold specified motor to hold so swing doesnt cause drift
        if (isLeft) {
            leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
        } else {
            rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
        }
        // cap outputs
        pidOutput = std::clamp(pidOutput, -maxSpeed, maxSpeed);

        // check which way the swing should turn
        if (isLeft) {
            tank(0, pidOutput);
        } else {
            tank(pidOutput, 0);
        }

        pros::delay(20);
    } while (!swingPID.isSettled());

    arcade(0, 0);
    distTravelled = -1;
    this->end_motion();
}

/**
 * @brief swings to a target angle with specific side of base. Exits with timeout or if PID is settled.
 *
 * @param heading target angle
 * @param isLeft true = moves to the target angle left, false = moves to the target angle right
 * @param degreeRange how many degrees the movement should exit in
 * @param maxSpeed max speed the robot should move at
 * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
 *
 */
void Chassis::swing_without_settle(float heading, bool isLeft, float degreeRange, float maxSpeed, bool async) {
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { swing_without_settle(heading, isLeft, degreeRange, maxSpeed, false); });
        this->end_motion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    swingPID.reset();
    // globalize error for exiting
    float error = 0;
    do {
        // calculate error
        error = rollAngle180(heading - imu->get_heading());
        // set distTravelled to error for async purposes
        distTravelled = error;
        // calculate pid output
        float pidOutput = swingPID.update(0, -error);
        // hold specified motor to hold so swing doesnt cause drift
        if (isLeft) {
            leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
        } else {
            rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
        }
        // cap max speeds
        pidOutput = std::clamp(pidOutput, -maxSpeed, maxSpeed);

        // check which way the swing should turn
        if (isLeft) {
            tank(0, pidOutput);
        } else {
            tank(pidOutput, 0);
        }
        // delay
        pros::delay(20);
    } while (!swingPID.isSettled() || fabs(error) > degreeRange);
    // coast motors
    leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    // set distTravelled to -1 to let async queue know
    distTravelled = -1;
    this->end_motion();
}

/**
 * @brief does a circular arc to target heading using a ratio on the PID outputs.
 *
 * @param heading target angle
 * @param leftMult 0< leftMult <1 ratio of left side pid output
 * @param rightMult 0<rightMult<1 ratio of right side pid output
 * @param maxSpeed max speed the robot should move at
 * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
 *
 */
void Chassis::arc(float heading, double leftMult, double rightMult, float maxSpeed, bool async) {
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { arc(heading, leftMult, rightMult, maxSpeed, false); });
        this->end_motion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    arcPID.reset();
    do {
        // calculate error
        float error = rollAngle180(heading - imu->get_heading());
        // set distTravelled to error for async purposes
        distTravelled = error;

        // calculate pid outputs
        float pidOutput = arcPID.update(0, -error);

        // cap speeds
        pidOutput = std::clamp(pidOutput, -maxSpeed, maxSpeed);

        // break if error is less than one
        if (fabs(error) < 1) break;
        // run
        tank(pidOutput * leftMult, pidOutput * rightMult);
        // delay
        pros::delay(10);
    } while (!arcPID.isSettled());
    // stop motors
    arcade(0, 0);
    // coast em
    leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    // set distTravelled to -1 to let async queue know
    distTravelled = -1;
    this->end_motion();
}

/**
 * @brief does a circular arc to target heading using a ratio on the PID outputs, doesnt settle(can link to next
   movement to make it fluid)
 *
 * @param heading target angle
 * @param leftMult 0< leftMult <1 ratio of left side pid output
 * @param rightMult 0<rightMult<1 ratio of right side pid output
 * @param maxSpeed max speed the robot should move at
 * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
 *
*/
void Chassis::arc_non_settle(float heading, double leftMult, double rightMult, float maxSpeed, bool async) {
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { arc_non_settle(heading, leftMult, rightMult, maxSpeed, false); });
        this->end_motion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    arcPID.reset();
    do {
        // calculate error
        float error = rollAngle180(heading - imu->get_heading());
        // set distTravelled for async purposes
        distTravelled = error;
        // calculate pid outputs
        float pidOutput = arcPID.update(0, -error);
        // cap max speeds
        pidOutput = std::clamp(pidOutput, -maxSpeed, maxSpeed);
        //break if error is less than one
        if (fabs(error) < 1) break;
        // run
        tank(pidOutput * leftMult, pidOutput * rightMult);
        // delay
        pros::delay(10);
    } while (!arcPID.isSettled());
    // coast em
    leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    // set distTravelled to -1 to let async queue know
    distTravelled = -1;
    this->end_motion();
}

/**
 * @brief uses a preset radius to move the robot to a certain angle using radius(in inches)
 *
 * @param heading target angle
 * @param radius imaginary radius distance from a point that the robot is circling around
 * @param maxSpeed max speed the robot should move at
 * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
 *
 */
void Chassis::radius_arc(float heading, float radius, float maxSpeed, bool async) {
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { radius_arc(heading, radius, maxSpeed, false); });
        this->end_motion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    arcPID.reset();
    headingPID.reset();
    bool reverse = radius < 0;
    radius = fabs(radius);

    double deltaTheta = deltaInHeading(heading, imu->get_heading());

    double totalDistance = fabs(deltaTheta) * radius;
    double HTW = trackWidth / 2.0;
    double slowerWheelRatio = (radius - HTW) / (radius + HTW);

    double largerDistanceTotal = (radius + HTW) * fabs(deltaTheta);
    auto start = pros::millis();

    while (!arcPID.isSettled()) {
        double largerDistanceCurrent = (deltaTheta > 0 != reverse)
                                           ? rightMotors->get_positions()[0] * wheelDiameter * M_PI * gearRatio
                                           : leftMotors->get_positions()[0] * wheelDiameter * M_PI * gearRatio;
        largerDistanceCurrent = fabs(largerDistanceCurrent);
        double distanceError = largerDistanceTotal - largerDistanceCurrent;

        double fasterWheelSpeed = arcPID.update(distanceError, 0);
        double slowerWheelSpeed = fasterWheelSpeed * slowerWheelRatio;

        double targetTheta = deltaTheta * (largerDistanceCurrent / largerDistanceTotal);

        double headingError = rollAngle180(deltaInHeading(targetTheta, imu->get_heading()));
        distTravelled = headingError;
        double headingCorrection = headingPID.update(0, -headingError);

        double left, right;
        if (deltaTheta < 0 != reverse) {
            left = fasterWheelSpeed;
            right = slowerWheelSpeed;
        } else {
            left = slowerWheelSpeed;
            right = fasterWheelSpeed;
        }

        if (reverse) {
            left = -1;
            right = -1;
        }

        // IMU PID Correction:
        left -= headingCorrection;
        right += headingCorrection;

        tank(left, right);

        pros::delay(10);
    }

    tank(0, 0);
    distTravelled = -1;
    this->end_motion();
}

void Chassis::follow_path(std::vector<Pose> pPath, float targetLinVel, float targetAngVel, float timeOut,
                          float errorRange, float beta, float zeta, bool reversed) {
    float offFromPose = INT_MAX;

    // set up the timer
    auto start = pros::millis();
    float runtime = 0;

    // initialise loop variables
    int prevCloseIndex = 0;

    // keep running the controller until either time expires or the bot is within the error range
    while (pros::millis() - start < timeOut && offFromPose >= errorRange) {
        // find the closest index
        int closeIndex = FindClosest(odomPose, pPath, prevCloseIndex);

        // get the closest pose velocities
        Pose closestPose = pPath.at(closeIndex);
        float targetAngularVelocity = targetAngVel;
        float targetLinearVelocity = targetLinVel;

        // set the desired pose to one ahead (so the robot is always moving forward) *****TEST******
        int targetIndex = std::min(closeIndex + 1, (int)pPath.size() - 1); // ensures no out of range error
        Pose targetPose = pPath.at(targetIndex);

        // run the controller function
        ramsete(targetPose, odomPose, targetAngularVelocity, targetLinearVelocity, beta, zeta);

        pros::delay(10);
    }
}

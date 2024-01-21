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

Chassis::Chassis(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, pros::IMU* inertial,
                 const float wheelDiameter, const float trackWidth, const float gearRatio, PID drivePID,
                 PID backwardPID, PID turnPID, PID smallturnPID, PID swingPID, PID arcPID, PID headingPID)
    : leftMotors(leftMotors), rightMotors(rightMotors), imu(inertial), wheelDiameter(wheelDiameter),
      trackWidth(trackWidth), gearRatio(gearRatio), drivePID(drivePID), backwardPID(backwardPID), turnPID(turnPID),
      smallturnPID(smallturnPID), swingPID(swingPID), arcPID(arcPID), headingPID(headingPID) {
    this->leftMotors->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    this->rightMotors->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
}

Pose odomPose(0, 0, 0);

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

void Chassis::set_heading(float heading) { imu->set_heading(heading); }

void Chassis::set_pose(float x1, float y1, float theta1) {
    x = x1;
    y = y1;
    imu->set_heading(theta1);
}



void Chassis::move(float distance, float maxSpeed, bool async) {
    // dummy PID
    PID drivingPID = PID(0, 0, 0, 0, 0, 0, 0, 0, 0);
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
        PID drivingPID = drivePID;
    } else {
        PID drivingPID = backwardPID;
    }
    drivingPID.reset();
    float beginningLeft = leftMotors->get_positions()[0];
    float beginningRight = rightMotors->get_positions()[0];
    float angle = imu->get_heading();

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
        if (pidOutputLateral < -maxSpeed) { pidOutputLateral = -maxSpeed; }
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

void Chassis::move_without_settle(float distance, float exitrange, float maxSpeed, bool async) {
    // dummy PID
    PID drivingPID = PID(0, 0, 0, 0, 0, 0, 0, 0, 0);
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
        PID drivingPID = drivePID;
    } else {
        PID drivingPID = backwardPID;
    }
    drivingPID.reset();
    float beginningLeft = leftMotors->get_positions()[0];
    float beginningRight = rightMotors->get_positions()[0];
    float angle = imu->get_heading();
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
        if (pidOutputLateral < -maxSpeed) { pidOutputLateral = -maxSpeed; }
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

void Chassis::move_without_settletime(float distance, float timeout, float maxSpeed, bool async) {
    // dummy PID
    PID selectedPID = PID(0, 0, 0, 0, 0, 0, 0, 0, 0);
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
        PID selectedPID = drivePID;
    } else {
        PID selectedPID = backwardPID;
    }
    selectedPID.reset();
    // get prev values
    float beginningLeft = leftMotors->get_positions()[0];
    float beginningRight = rightMotors->get_positions()[0];
    float angle = imu->get_heading();

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
        if (pidOutputLateral < -maxSpeed) { pidOutputLateral = -maxSpeed; }
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
        if (pidOutput > maxSpeed) {
            pidOutput = maxSpeed;
        } else if (pidOutput < -maxSpeed) {
            pidOutput = -maxSpeed;
        } else {
            pidOutput = pidOutput;
        }
        // run
        arcade(0, pidOutput);
        pros::delay(20);
    } while (!selectedPID.isSettled());

    arcade(0, 0);
    distTravelled = -1;
    this->end_motion();
}

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
        //calculate error
        float error = rollAngle180(heading - imu->get_heading());
        //set distTravelled to error for async purposes
        distTravelled = error;
        //calculate pidOutput
        float pidOutput = swingPID.update(0, -error);
        //hold specified motor to hold so swing doesnt cause drift
        if(isLeft){
            leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
        } else{
            rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
        }
        //cap outputs
        if (pidOutput > maxSpeed) {
            pidOutput = maxSpeed;
        } else if (pidOutput < -maxSpeed) {
            pidOutput = -maxSpeed;
        } else {
            pidOutput = pidOutput;
        }
        //check which way the swing should turn
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

void Chassis::swing_without_settle(float heading, bool isLeft, float degreeRange, float maxSpeed, bool async) {
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { swing_without_settle(heading, isLeft, degreeRange,maxSpeed, false); });
        this->end_motion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    swingPID.reset();
    //globalize error for exiting
    float error = 0;
    do {
        //calculate error
        error = rollAngle180(heading - imu->get_heading());
        //set distTravelled to error for async purposes
        distTravelled = error;
        //calculate pid output
        float pidOutput = swingPID.update(0, -error);
        //hold specified motor to hold so swing doesnt cause drift
        if(isLeft){
            leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
        } else{
            rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
        }
        //cap max speeds
        if (pidOutput > maxSpeed) {
            pidOutput = maxSpeed;
        } else if (pidOutput < -maxSpeed) {
            pidOutput = -maxSpeed;
        } else {
            pidOutput = pidOutput;
        }
        //check which way the swing should turn
        if (isLeft) {
            tank(0, pidOutput);
        } else {
            tank(pidOutput, 0);
        }
        //delay
        pros::delay(20);
    } while (!swingPID.isSettled() || fabs(error) > degreeRange);
    //coast motors
    leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    //set distTravelled to -1 to let async queue know
    distTravelled = -1;
    this->end_motion();
}

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
        //calculate error
        float error = rollAngle180(heading - imu->get_heading()); 
        //set distTravelled to error for async purposes
        distTravelled = error;
        //calculate pid outputs
        float pidOutput = arcPID.update(0, -error);
        //cap speeds
        if (pidOutput > maxSpeed) pidOutput = maxSpeed;
        if (pidOutput < -maxSpeed) pidOutput = -maxSpeed;
        //break if error is less than one
        if (fabs(error) < 1) break;
        //run
        tank(pidOutput * leftMult, pidOutput * rightMult);
        //delay
        pros::delay(10);
    } while (!arcPID.isSettled());
    //stop motors
    arcade(0, 0);
    //coast em
    leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    //set distTravelled to -1 to let async queue know
    distTravelled = -1;
    this->end_motion();
}

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
        //calculate error
        float error = rollAngle180(heading - imu->get_heading());
        //set distTravelled for async purposes
        distTravelled = error;
        //calculate pid outputs
        float pidOutput = arcPID.update(0, -error);
        //cap max speeds
        if (pidOutput > maxSpeed) pidOutput = maxSpeed;
        if (pidOutput < -maxSpeed) pidOutput = -maxSpeed;
        if (fabs(error) < 1) break;
        //run
        tank(pidOutput * leftMult, pidOutput * rightMult);
        //delay
        pros::delay(10);
    } while (!arcPID.isSettled());
    //coast em
    leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    //set distTravelled to -1 to let async queue know
    distTravelled = -1;
    this->end_motion();
}

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
        if (motorPower > maxSpeed) motorPower = maxSpeed;
        else if (motorPower < -maxSpeed) motorPower = -maxSpeed;

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

void Chassis::move_to_pose(float x1, float y1, float theta1, int timeout, bool forwards, float maxSpeed, bool async,
                           float chasePower, float lead, float smoothness) {
    this->request_motion_start();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task(
            [&]() { move_to_pose(x1, y1, theta1, timeout, forwards, maxSpeed, false, chasePower, lead, smoothness); });
        this->end_motion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    turnPID.reset();
    drivePID.reset();

    double targetTheta = M_PI_2 - degToRad(theta1);
    float prevLateralPower = 0;
    float prevAngularPower = 0;

    // last pose
    float lastposex = x;
    float lastposey = y;
    float lastposetheta = heading;

    auto start = pros::millis();
    if (!forwards) targetTheta = fmod(targetTheta + M_PI, 2 * M_PI); // backwards movement

    bool close = false;
    if (chasePower == 0) chasePower = 40; // make chasePower globalized in chassis setup
    while ((!drivePID.isSettled() || pros::millis() - start < timeout)) {
        double currX = x;
        double currY = y;
        double currHeading = heading; // for reference
        double currTheta = degToRad(heading);

        if (!forwards) currTheta += M_PI;
        distTravelled += distance(lastposex, lastposey, currX, currY);

        // update prev
        lastposex = currX;
        lastposey = currY;
        lastposetheta = currHeading;

        if (distance(x1, y1, x, y) < 7.5) { close = true; }

        // carrot - 2 times for each part
        double carrotX = x1 - (cos(targetTheta) * lead * distance(x1, y1, currX, currY));
        double carrotY = y1 - (sin(targetTheta) * lead * distance(x1, y1, currX, currY));
        if (close) { // settle behavior
            x1 = carrotX;
            y1 = carrotY;
        }

        // calculate error
        float angularError =
            angleError(pointAngleDifference(carrotX, carrotY, currX, currY), currTheta, true); // angular error
        float linearError = distance(carrotX, carrotY, currX, currY) * cos(angularError); // linear error

        if (close) angularError = angleError(targetTheta, currTheta, true); // settling behavior
        if (!forwards) linearError = -linearError;

        // get PID outputs
        float angularPower = -turnPID.update(radToDeg(angularError), 0);
        float linearPower = drivePID.update(linearError, 0);

        float curvature = fabs(getCurvature(currX, currY, currTheta, carrotX, carrotY, 0));
        if (curvature == 0) curvature = -1;
        float radius = 1 / curvature;

        // calculate the maximum speed at which the robot can turn
        // using the formula v = sqrt( u * r * g )
        if (radius != -1) {
            float maxTurnSpeed = sqrt(chasePower * radius * 9.8);
            // the new linear power is the minimum of the linear power and the max turn speed
            if (linearPower > maxTurnSpeed && !close) linearPower = maxTurnSpeed;
            else if (linearPower < -maxTurnSpeed && !close) linearPower = -maxTurnSpeed;
        }
        // prioritize turning over moving
        float overturn = fabs(angularPower) + fabs(linearPower) - maxSpeed - maxSpeed * smoothness;

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

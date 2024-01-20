#include "chassis.hpp"
#include "Eigen/Eigen"
#include "pros/motors.hpp"
#include "pros/llemu.hpp"
#include "api.h"
#include <cmath>
#include <iostream>

#define M_PI		3.14159265358979323846
#define M_PI_2		1.57079632679489661923
#define M_PI_4		0.78539816339744830962
#define M_1_PI		0.31830988618379067154
#define M_2_PI		0.63661977236758134308

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
 * @param smallTurnPID PID used for smaller turn movements; you can set this to the same as the turnPID if you want, this is used to fine tune small turns.
 * @param swingPID PID used for swing movements
 * @param arcPID PID used for arc movements
 * @param headingPID PID used to keep the robot at a certain heading during movements
 */
Chassis::Chassis(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, pros::IMU* inertial, const float wheelDiameter, const float trackWidth,
                 const float gearRatio, PID drivePID, PID backwardPID, PID turnPID, PID smallturnPID,PID swingPID, PID arcPID, PID headingPID)
    : leftMotors(leftMotors), rightMotors(rightMotors), imu(inertial), wheelDiameter(wheelDiameter),trackWidth(trackWidth), gearRatio(gearRatio),
      drivePID(drivePID), backwardPID(backwardPID), turnPID(turnPID), smallturnPID(smallturnPID), swingPID(swingPID), arcPID(arcPID),headingPID(headingPID){
    this->leftMotors->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    this->rightMotors->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
}


/*========================================================INTIALIZE----------------------------------------------------------*/
Pose odomPose(0,0,0);
/**
    * @brief calibrate chassis
*/
void Chassis::calibrate() {
    imu->reset();
    leftMotors->tare_position();
    rightMotors->tare_position();
    
    pros::Task([=]{
        double prevLeft = 0;
        double prevRight = 0;
        double prevTheta= 0;
        x = 0;
        y = 0;
        heading = 0;
        
        while(true){
            double left = leftMotors->get_positions()[0]* wheelDiameter * M_PI * gearRatio;
            double right = rightMotors->get_positions()[0]* wheelDiameter * M_PI * gearRatio;;
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
void Chassis::setHeading(float heading) {
    imu->set_heading(heading);
}
/**
         * @brief sets the odometry pose of the bot
         * 
         * @param x1 set the x value
         * @param y1 set the y value
         * @param theta1 set the theta value(in degrees)
        */

void Chassis::setPose(float x1, float y1, float theta1) {
    x = x1;
    y = y1;
    imu->set_heading(theta1);
}

/**
         * @brief moves the bot using tank fashion(left controls leftside of the base, right controls rightside of the base)
         * 
         * @param left left power
         * @param right right power
        */
void Chassis::tank(float left, float right) {
    leftMotors->move(left);
    rightMotors->move(right);
}
/**
         * @brief moves the bot using arcade fashion(lateral controls forward/backward movements while angular puts turn bias on the powers)
         * 
         * @param lateral left power
         * @param angular right power
        */
void Chassis::arcade(float lateral, float angular) {
    double leftmotorsmove = lateral + angular;
    double rightmotorsmove = lateral - angular;
    leftMotors->move(leftmotorsmove);
    rightMotors->move(rightmotorsmove);
}
/*========================================================ASYNC ACTION----------------------------------------------------------*/
void Chassis::waitUntil(float error) {
    // do while to give the thread time to start
    do pros::delay(10);
    while (distTravelled <= error && distTravelled != -1);
} 
void Chassis::endMotion() {
    // move the "queue" forward 1
    this->motionRunning = this->motionQueued;
    this->motionQueued = false;

    // permit queued motion to run
    this->mutex.give();
}
void Chassis::waitUntilDone() {
    do pros::delay(10);
    while (distTravelled != -1);
}
void Chassis::cancelMotion() {
    this->motionRunning = false;
    pros::delay(10); // give time for motion to stop
}
void Chassis::requestMotionStart() {
    if (this->isInMotion()) this->motionQueued = true; // indicate a motion is queued
    else this->motionRunning = true; // indicate a motion is running

    // wait until this motion is at front of "queue"
    this->mutex.take(TIMEOUT_MAX);

    // this->motionRunning should be true
    // and this->motionQueued should be false
    // indicating this motion is running
}
void Chassis::cancelAllMotions() {
    this->motionRunning = false;
    this->motionQueued = false;
    pros::delay(10); // give time for motion to stop
}

bool Chassis::isInMotion() const { return this->motionRunning; }




/*----------------------------------------------------REGULAR MOVE FUNCTION--------------------------------------------------------------*/
/**
         * @brief moves the bot forward using forwards or backwards PID
         * 
         * @param distance distance the bot should move to(in inches)
         * @param maxSpeed the max speed the robot should travel in(out of 127). 
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
        */
void Chassis::move(float distance, float maxSpeed,bool async) {
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { move(distance, maxSpeed, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    drivePID.reset();
    backwardPID.reset();
    headingPID.reset();
    float beginningLeft = leftMotors->get_positions()[0];
    float beginningRight = rightMotors->get_positions()[0];
    float angle = imu->get_heading();
    if(distance < 0){
        do {
        
        float deltaLeft = leftMotors->get_positions()[0] - beginningLeft;
        float deltaRight = rightMotors->get_positions()[0] - beginningRight;
        float distanceTravelled = (deltaLeft + deltaRight) / 2 * wheelDiameter * M_PI * gearRatio;
        distTravelled = distanceTravelled;
        float error = rollAngle180(angle - imu->get_heading());
        float pidAngOutput = headingPID.update(0, -error);
        float pidOutputLateral = backwardPID.update(distance, distanceTravelled);
        if (pidOutputLateral<-maxSpeed) {
            pidOutputLateral=-maxSpeed; 
        }
        arcade(pidOutputLateral, pidAngOutput);

        pros::delay(20);
    } while (!backwardPID.isSettled());
    } else{
        do {
         
        float deltaLeft = leftMotors->get_positions()[0] - beginningLeft;
        float deltaRight = rightMotors->get_positions()[0] - beginningRight;
        float distanceTravelled = (deltaLeft + deltaRight) / 2 * wheelDiameter * M_PI * gearRatio;
        distTravelled = distanceTravelled;
        float error = rollAngle180(angle - imu->get_heading());
        float pidAngOutput = headingPID.update(0, -error);
        float pidOutputLateral = drivePID.update(distance, distanceTravelled);
        if (pidOutputLateral>maxSpeed) {
            pidOutputLateral=maxSpeed; 
        }
        arcade(pidOutputLateral, pidAngOutput);

        pros::delay(20);
    } while (!drivePID.isSettled());
    }
    arcade(0, 0);
    distTravelled = -1;
    this->endMotion();
}
/**
         * @brief moves the bot forwards or backwards using the forward or backward PID and exits into the user specified next movement
         * 
         * @param distance distance the bot should move to(in inches)
         * @param exitrange how much distance the robot should settle in(ie. want to move 24 inches: distance = 26, exitrange = 2)
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
        */
void Chassis::move_without_settle(float distance, float exitrange,bool async){
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&](){move_without_settle(distance, exitrange, false);});
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    drivePID.reset();
    float beginningLeft = leftMotors->get_positions()[0];
    float beginningRight = rightMotors->get_positions()[0];
    float error;
    auto start = pros::millis();
    do {
        
        float deltaLeft = leftMotors->get_positions()[0] - beginningLeft;
        float deltaRight = rightMotors->get_positions()[0] - beginningRight;
        float distanceTravelled = (deltaLeft + deltaRight) / 2 * wheelDiameter * M_PI * gearRatio;
        distTravelled = distanceTravelled;
        error = distance-distanceTravelled;
        float pidOutput = drivePID.update(distance, distanceTravelled);
        if(fabs(error) <= exitrange){
            break;
        }
        arcade(pidOutput, 0);

        pros::delay(20);
    } while (!drivePID.isSettled() || fabs(error) > exitrange); 
    arcade(0,0);
    leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    distTravelled = -1;
    this->endMotion();
}
/**
         * @brief moves the bot forwards or backwards using the forward or backward PID and exits after a certain time is reached
         * 
         * @param distance distance the bot should move to(in inches)
         * @param timeout how much time the robot should move until 
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
        */
void Chassis::move_without_settletime(float distance, float timeout,bool async){
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&](){move_without_settletime(distance, timeout, false);});
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    drivePID.reset();

    float beginningLeft = leftMotors->get_positions()[0];
    float beginningRight = rightMotors->get_positions()[0];

    auto start = pros::millis();
    do {
        
        float deltaLeft = leftMotors->get_positions()[0] - beginningLeft;
        float deltaRight = rightMotors->get_positions()[0] - beginningRight;
        float distanceTravelled = (deltaLeft + deltaRight) / 2 * wheelDiameter * M_PI * gearRatio;
        distTravelled = distanceTravelled;
        float pidOutput = drivePID.update(distance, distanceTravelled);
        arcade(pidOutput, 0);

        pros::delay(20);
    } while ((pros::millis() - start) < timeout);
    arcade(0,0);
    leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    distTravelled = -1;
    this->endMotion();
}
/**
         * @brief moves the bot forwards or backwards using the forward or backward PID while approaching with the target angle
         * 
         * @param distance distance the bot should move to(in inches)
         * @param heading the target heading the robot should approach while moving
         * @param maxSpeed the max speed the robot should travel in(out of 127). 
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
        */
void Chassis::movewithheading(float distance, float heading, float maxSpeed,bool async){
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&](){movewithheading(distance, heading, maxSpeed, false);});
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    drivePID.reset();
    backwardPID.reset();
    headingPID.reset();
    float beginningLeft = leftMotors->get_positions()[0];
    float beginningRight = rightMotors->get_positions()[0];
    float angle = heading;
    if(distance < 0){
        do {
        
        float deltaLeft = leftMotors->get_positions()[0] - beginningLeft;
        float deltaRight = rightMotors->get_positions()[0] - beginningRight;
        float distanceTravelled = (deltaLeft + deltaRight) / 2 * wheelDiameter * M_PI * gearRatio;
        distTravelled = distanceTravelled;
        float error = rollAngle180(angle - imu->get_heading());
        float pidAngOutput = headingPID.update(0, -error);
        float pidOutputLateral = backwardPID.update(distance, distanceTravelled);
        if (pidOutputLateral<-maxSpeed) {
            pidOutputLateral=-maxSpeed; 
        }
        arcade(pidOutputLateral, pidAngOutput);

        pros::delay(20);
    } while (!backwardPID.isSettled());
    } else{
        do {
       
        float deltaLeft = leftMotors->get_positions()[0] - beginningLeft;
        float deltaRight = rightMotors->get_positions()[0] - beginningRight;
        float distanceTravelled = (deltaLeft + deltaRight) / 2 * wheelDiameter * M_PI * gearRatio;
        distTravelled = distanceTravelled;
        float error = rollAngle180(angle - imu->get_heading());
        float pidAngOutput = headingPID.update(0, -error);
        float pidOutputLateral = drivePID.update(distance, distanceTravelled);
        if (pidOutputLateral>maxSpeed) {
            pidOutputLateral=maxSpeed; 
        }
        arcade(pidOutputLateral, pidAngOutput);

        pros::delay(20);
    } while (!drivePID.isSettled());
    }
    arcade(0, 0);
    distTravelled = -1;
    this->endMotion();
}
/*----------------------------------------------------TURN FUNCTIONS-------------------------------------------------------------------------*/
/**
         * @brief turns the robot to a target angle
         * 
         * @param heading target angle
         * @param maxSpeed max speed the robot should move at
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
         
         * 
        */
void Chassis::turn(float heading,float maxSpeed,bool async) {
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&](){turn(heading, maxSpeed, false);});
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    turnPID.reset();

    do {
        
        float error = rollAngle180(heading - imu->get_heading());
        distTravelled = error;
        float pidOutput = turnPID.update(0, -error);
        if (pidOutput>maxSpeed) {
            pidOutput=maxSpeed; 
        } else if (pidOutput<-maxSpeed) {
            pidOutput=-maxSpeed; 
        } else {
            pidOutput = pidOutput;
        }
        arcade(0, pidOutput);

        pros::lcd::print(1, "heading: %f", imu->get_heading());

        pros::delay(20);
    } while (!turnPID.isSettled());
    arcade(0, 0);
    distTravelled = -1;
    this->endMotion();
}
/**
         * @brief turns the robot to a target angle using small turn PID.
         * 
         * @param heading target angle
         * @param maxSpeed max speed the robot should move at
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
         
         * 
        */
void Chassis::turnsmall(float heading, float maxSpeed,bool async){
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&](){turnsmall(heading, maxSpeed, false);});
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    smallturnPID.reset();

    do {
        
        float error = rollAngle180(heading - imu->get_heading());
        distTravelled = error;
        float pidOutput = smallturnPID.update(0, -error);
        if (pidOutput>maxSpeed) {
            pidOutput=maxSpeed; 
        } else if (pidOutput<-maxSpeed) {
            pidOutput=-maxSpeed; 
        } else {
            pidOutput = pidOutput;
        }
        arcade(0, pidOutput);

        pros::lcd::print(1, "heading: %f", imu->get_heading());

        pros::delay(20);
    } while (!smallturnPID.isSettled());

    arcade(0, 0);
    distTravelled = -1;
    this->endMotion();
}

/*-------------------------------------------SWING FUNCTIONS----------------------------------------------------------------*/
/**
         * @brief swings to a target angle with specific side of base
         * 
         * @param heading target angle
         * @param isLeft true = moves to the target angle left, false = moves to the target angle right
         * @param maxSpeed max speed the robot should move at
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
         
         * 
        */
void Chassis::swing(float heading, bool isLeft, float maxSpeed,bool async){
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&](){swing(heading, isLeft, maxSpeed, false);});
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    swingPID.reset();
    if(isLeft){
        do {
       
        float error = rollAngle180(heading - imu->get_heading());
        distTravelled = error;
        float pidOutput = swingPID.update(0, -error);
        Chassis::rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
        if (pidOutput>maxSpeed) {
            pidOutput=maxSpeed; 
        } else if (pidOutput<-maxSpeed) {
            pidOutput=-maxSpeed; 
        } else {
            pidOutput = pidOutput;
        }
        tank(0,pidOutput);
        pros::lcd::print(1, "heading: %f", imu->get_heading());

        pros::delay(10);
    } while (!swingPID.isSettled());
    arcade(0, 0);
    distTravelled = -1;
    this->endMotion();
    } else{
        do {
        
        float error = rollAngle180(heading - imu->get_heading());
        distTravelled = error;
        float pidOutput = swingPID.update(0, -error);
        Chassis::leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
        if (pidOutput>maxSpeed) {
            pidOutput=maxSpeed; 
        } else if (pidOutput<-maxSpeed) {
            pidOutput=-maxSpeed; 
        } else {
            pidOutput = pidOutput;
        }
        tank(pidOutput, 0);

        pros::lcd::print(1, "heading: %f", imu->get_heading());

        pros::delay(10);
    } while (!swingPID.isSettled());
    arcade(0, 0);
    distTravelled = -1;
    this->endMotion();
    }
}

/**
         * @brief swings to a target angle with specific side of base. Exits with timeout or if PID is settled.
         * 
         * @param heading target angle
         * @param isLeft true = moves to the target angle left, false = moves to the target angle right
         * @param timeout how long the movement should take before exiting
         * @param maxSpeed max speed the robot should move at
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
         
         * 
        */
void Chassis::swing_without_settle(float heading, bool isLeft, float timeout,bool async){
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&](){swing(heading, isLeft, timeout, false);});
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    swingPID.reset();
    if(isLeft){
        auto start = pros::millis();
        do {
        
        float error = rollAngle180(heading - imu->get_heading());
        distTravelled = error;
        float pidOutput = swingPID.update(0, -error);
        Chassis::leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
        tank(0, pidOutput);
        if(fabs(error)< 1)break;
        pros::lcd::print(1, "heading: %f", imu->get_heading());

        pros::delay(10);
    } while ((pros::millis() - start) != timeout);
    leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    distTravelled = -1;
    this->endMotion();
    } else{
        auto start = pros::millis();
        do {
        
        float error = rollAngle180(heading - imu->get_heading());
        distTravelled = error;
        float pidOutput = swingPID.update(0, -error);
        Chassis::rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
        tank(pidOutput, 0);
        if(fabs(error)<1)break;
        pros::lcd::print(1, "heading: %f", imu->get_heading());

        pros::delay(10);
    } while ((pros::millis() - start) != timeout);
    leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    distTravelled = -1;
    this->endMotion();
    }
}

/*-----------------------------------------------------ARC FUNCTIONS------------------------------------------------------*/
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
void Chassis::arc(float heading, double leftMult, double rightMult, float maxSpeed,bool async){
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&](){arc(heading, leftMult, rightMult, maxSpeed,false);});
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    arcPID.reset();
        do {
        
        float error = rollAngle180(heading - imu->get_heading());
        distTravelled = error;
        float pidOutput = arcPID.update(0, -error);
        if(pidOutput > maxSpeed) pidOutput = maxSpeed;
        if(pidOutput <-maxSpeed) pidOutput = -maxSpeed;
        if(fabs(error)< 1)break;
        tank(pidOutput * leftMult, pidOutput * rightMult);
        
        pros::lcd::print(1, "heading: %f", imu->get_heading());

        pros::delay(10);
    } while (!arcPID.isSettled());
    arcade(0,0);
    leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    distTravelled = -1;
    this->endMotion();
    }
/**
         * @brief does a circular arc to target heading using a ratio on the PID outputs, doesnt settle(can link to next movement to make it fluid)
         * 
         * @param heading target angle
         * @param leftMult 0< leftMult <1 ratio of left side pid output
         * @param rightMult 0<rightMult<1 ratio of right side pid output
         * @param maxSpeed max speed the robot should move at
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
         
         * 
        */
void Chassis::arcnonsettle(float heading, double leftMult, double rightMult, float maxSpeed,bool async){
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&](){arcnonsettle(heading, leftMult, rightMult, maxSpeed,false);});
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    arcPID.reset();
        do {
        
        float error = rollAngle180(heading - imu->get_heading());
        distTravelled = error;
        float pidOutput = arcPID.update(0, -error);
        if(pidOutput > maxSpeed) pidOutput = maxSpeed;
        if(pidOutput <-maxSpeed) pidOutput = -maxSpeed;
        if(fabs(error)< 1)break;
        tank(pidOutput * leftMult, pidOutput * rightMult);
        
        pros::lcd::print(1, "heading: %f", imu->get_heading());

        pros::delay(10);
    } while (!arcPID.isSettled());
    
    leftMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    rightMotors->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    distTravelled = -1;
    this->endMotion();
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
void Chassis::radiusarc(float heading, float radius, float maxSpeed, bool async){
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&](){radiusarc(heading, radius, maxSpeed,false);});
        this->endMotion();
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
        
        double largerDistanceCurrent = (deltaTheta > 0 != reverse) ? rightMotors->get_positions()[0] * wheelDiameter * M_PI * gearRatio : leftMotors->get_positions()[0] * wheelDiameter * M_PI * gearRatio;
        largerDistanceCurrent = fabs(largerDistanceCurrent);
        double distanceError = largerDistanceTotal - largerDistanceCurrent;
        
        double fasterWheelSpeed = arcPID.update(distanceError,0);
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
            right= -1;
        }

        // IMU PID Correction:
        left -= headingCorrection; 
        right += headingCorrection;

        tank(left, right);

        pros::delay(10);
    }

    tank(0,0);
    distTravelled = -1;
    this->endMotion();
}
/*------------------------------------------------------ODOM MOVEMENTS------------------------------------------------------------*/
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
void Chassis::turnToPoint(float x1, float y1, int timeout, bool forwards, float maxSpeed,bool async){
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { turnToPoint(x1, y1, timeout, forwards, maxSpeed, false); });
        this->endMotion();
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
        distTravelled += pose.distance(Pose(x1,y1));
        // update completion vars
        distTravelled = fabs(angleError(pose.theta, startTheta, false));

        deltaX = x - pose.x;
        deltaY = y - pose.y;
        targetTheta = fmod(radToDeg(M_PI_2 - atan2(deltaY, deltaX)), 360);

        // calculate deltaTheta
        deltaTheta = angleError(targetTheta, pose.theta, false);

        // calculate the speed
        motorPower = turnPID.update(0,-deltaTheta);

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
    this->endMotion();
    
}
 /**
         * @brief moves the bot to a certain point on the field
         * 
         * @param x1 target x coordinate in inches
         * @param y1 target y coordinate in inches
         * @param timeout target y coordinate in inches
         * @param maxSpeed the max speed the robot should travel in(out of 127). 
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
        */
void Chassis::moveToPoint(float x1, float y1, int timeout, float maxSpeed,bool async){
    turnPID.reset();
    drivePID.reset();
    
    float prevLateralPower = 0;
    float prevAngularPower = 0;
    bool close = false;
    uint32_t start = pros::millis();
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { moveToPoint(x1,y1, timeout, maxSpeed, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    while(((start < timeout) || (!drivePID.isSettled() && !turnPID.isSettled()))){
        heading = std::fmod(heading, 360);
        distTravelled += 10;
        //update error
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
    tank(0,0);
    distTravelled = -1;
    this->endMotion();

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
         * @param chasePower how fast the robot should go during the movement. If its higher, the accuracy of the movement is less.
         * @param lead 0 < lead < 1 how much the movement should curve
         * @param smoothness 0 < smoothness < 1 how smooth the movement should be. 
         * 
        */
void Chassis::moveToPose(float x1, float y1, float theta1, int timeout, bool forwards, float maxSpeed, bool async,float chasePower,
                          float lead, float smoothness) {
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { moveToPose(x1, y1, theta1, timeout, forwards, maxSpeed, false, chasePower, lead, smoothness); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    turnPID.reset();
    drivePID.reset();
    
    double targetTheta = M_PI_2 - degToRad(theta1);
    float prevLateralPower = 0;
    float prevAngularPower = 0;

    //last pose
    float lastposex = x;
    float lastposey = y;
    float lastposetheta = heading;

    auto start = pros::millis();
    if (!forwards) targetTheta = fmod(targetTheta + M_PI, 2 * M_PI); // backwards movement

    bool close = false;
    if(chasePower ==  0)chasePower = 40; // make chasePower globalized in chassis setup
    while ((!drivePID.isSettled() || pros::millis() - start < timeout)) {
        double currX = x;
        double currY = y;
        double currHeading = heading; //for reference
        double currTheta = degToRad(heading);

        if (!forwards) currTheta += M_PI;
        distTravelled += distance(lastposex, lastposey, currX, currY); 

        //update prev
        lastposex = currX;
        lastposey = currY;
        lastposetheta = currHeading;

        if (distance(x1, y1, x, y) < 7.5) {
            close = true;
        }

        //carrot - 2 times for each part
        double carrotX = x1 - (cos(targetTheta) * lead * distance( x1, y1,currX, currY));
        double carrotY = y1 - (sin(targetTheta) * lead * distance( x1, y1,currX, currY));   
        if(close){ //settle behavior
            x1 = carrotX;
            y1 = carrotY;
        }

        // calculate error
        float angularError = angleError(pointAngleDifference(carrotX, carrotY, currX, currY), currTheta, true); // angular error
        float linearError = distance(carrotX, carrotY,currX, currY) * cos(angularError); // linear error
        
        if (close) angularError = angleError(targetTheta, currTheta, true); // settling behavior
        if (!forwards) linearError = -linearError;
        
        // get PID outputs
        float angularPower = -turnPID.update(radToDeg(angularError), 0);
        float linearPower = drivePID.update(linearError, 0);
    
        
        float curvature = fabs(getCurvature(currX,currY, currTheta, carrotX, carrotY, 0));
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
    
    tank(0,0);
    distTravelled = -1;
    this->endMotion();
}


//ramesete
void Chassis::moveChassis(float linearVelocity, float angularVelocity) {
    // compute left and right velocities
    float leftVelocity = (2 * linearVelocity - angularVelocity * trackWidth) / 2; // inches/sec
    float rightVelocity = (2 * linearVelocity + angularVelocity * trackWidth) / 2; // inches/sec

    // calculate left and right wheel rpm
    float leftRPM = leftVelocity * 60.0 / (wheelDiameter * M_PI); // rpm
    float rightRPM = rightVelocity * 60.0 / (wheelDiameter * M_PI); // rpm

    // calculate the left and right motor rpm
    float leftMotorRPM = leftRPM * (gearRatio); //gearset
    float rightMotorRPM = rightRPM * (gearRatio); //gearset

    // move chassis
    leftMotors->move_velocity(leftMotorRPM);
    rightMotors->move_velocity(rightMotorRPM);
}

void Chassis::ramsete(Pose targetPose, Pose currentPose, float targetAngularVelocity, float targetLinearVelocity, float beta, float zeta) {
    // compute global error
    Eigen::MatrixXd globalError(1, 3);
    globalError <<
        targetPose.x - currentPose.x,
        targetPose.y - currentPose.y,
        degToRad(targetPose.theta) - currentPose.theta;

    // compute transformation matrix
    Eigen::MatrixXd transformationMatrix(3, 3);
    transformationMatrix <<
        cos(currentPose.theta),  sin(currentPose.theta), 0,
        -sin(currentPose.theta), cos(currentPose.theta), 0,
        0,                       0,                      1;

    // compute local error
    Eigen::MatrixXd localError = globalError * transformationMatrix;
    // compute k gain
    float k = 2 * zeta * std::sqrt(targetAngularVelocity * targetAngularVelocity + beta + targetLinearVelocity * targetLinearVelocity);
    // compute angular velocity
    float angularVelocity = targetAngularVelocity * cos(localError(0, 2)) + k * localError(0, 0);
    // compute linear velocity
    float linearVelocity = targetLinearVelocity + k * localError(0, 2) + (beta * linearVelocity * sin(localError(0, 2)) * localError(0, 1) / localError(0, 2));

    // move chassis
    moveChassis(linearVelocity, angularVelocity);
} // works

int Chassis::FindClosest(Pose pose, std::vector<Pose> pPath, int prevCloseIndex) {
    //Find the closest point to the robot
    int closeIndex = 0;
    float minDistance = INT_MAX;
    for(int i = prevCloseIndex; i<pPath.size(); i++){
        float dist = pose.distance(pPath.at(i));
        if(dist < minDistance){
            closeIndex = i;
            minDistance = dist;
        }
    }
    return closeIndex;
}
/**
         * @brief follows path using ramsete(std::vector<Pose>{})
         * 
         * @param pPath list of points the robot should follow
         * @param targetLinVel in RPM, how fast the robot should move linear wise.
         * @param targetAngVel in radians/sec, how fast the angular velocity the robot should have during turns or curves
         * @param timeOut how long the entire movement should take
         * @param errorRange how much distance the robot can ignore before moving on to the next point
         * @param beta 

         * 
        */
void Chassis::followPath(std::vector<Pose> pPath, float targetLinVel, float targetAngVel, float timeOut, float errorRange, float beta, float zeta, bool reversed){
    float offFromPose = INT_MAX;
    
    // set up the timer
    auto start = pros::millis();
    float runtime = 0;

    // initialise loop variables
    int prevCloseIndex=0;

    // keep running the controller until either time expires or the bot is within the error range
    while(pros::millis() - start < timeOut && offFromPose >= errorRange){

        // find the closest index
        int closeIndex = FindClosest(odomPose, pPath, prevCloseIndex);

        // get the closest pose velocities
        Pose closestPose = pPath.at(closeIndex);
        float targetAngularVelocity = targetAngVel;
        float targetLinearVelocity = targetLinVel;
        
        // set the desired pose to one ahead (so the robot is always moving forward) *****TEST******
        int targetIndex = std::min(closeIndex+1, (int)pPath.size()-1); // ensures no out of range error
        Pose targetPose = pPath.at(targetIndex);

        // run the controller function
        ramsete(targetPose, odomPose, targetAngularVelocity, targetLinearVelocity, beta, zeta);

        pros::delay(10);
    }
}

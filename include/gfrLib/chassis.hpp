#pragma once

#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include "util/pose.hpp"
#include "util/pid.hpp"
#include "util/util.hpp"
#include <functional>
#include "util/gainScheduler.hpp"

namespace gfrLib {

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
struct pidSetup {
        pidSetup(PID drivePID, PID backwardPID, PID turnPID, PID smallturnPID, PID swingPID, PID arcPID, PID headingPID);

        PID drivePID;
        PID turnPID;
        PID smallturnPID;
        PID swingPID;
        PID headingPID;
        PID backwardPID;
        PID arcPID;
};

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
 * @param defaultChasePower the default chase power for the move to pose.
 * 
 */
struct driveTrain {
        driveTrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, pros::IMU* inertial,
                   const float wheelDiameter, const float trackWidth, const float gearRatio, const float defaultChasePower);

        pros::MotorGroup* leftMotors;
        pros::MotorGroup* rightMotors;
        pros::IMU* imu;
        const float wheelDiameter;
        const float trackWidth;
        const float gearRatio;
        const float defaultChasePower;
};

/**
 * @brief Parameters for Chassis::moveToPose
 *
 * We use a struct to simplify customization. Chassis::moveToPose has many
 * parameters and specifying them all just to set one optional param ruins
 * readability. By passing a struct to the function, we can have named
 * parameters, overcoming the c/c++ limitation
 *
 * @param forwards whether the robot should move forwards or backwards. True by default
 * @param maxSpeed the maximum speed the robot can travel at. Value between 0-127.
 *  127 by default
 * @param chasePower how fast the robot will move around corners. Recommended value 2-15.
 *  0 means use chasePower set in chassis class. 0 by default.
 * @param lead carrot point multiplier. value between 0 and 1. Higher values result in
 *  curvier movements. 0.6 by default
 * @param gLead ghost point weight to approach target smoothly
 * @param smoothness how smooth the movement should be @note slower, but approaches smaller targets more smoothly.
 * 
 */
struct MoveToPoseParams {
        bool forwards = true;
        float maxSpeed = 127;
        float chasePower = 0;
        float lead = 0.6;
        float gLead = 0;
        float smoothness = 1;

};

/**
 * @brief Parameters for gainScheduler kP tuner.
 *
 * @note this struct is used to tune kP live during the movement. 
 *
 * @param min the minimum speed the bot could be travelling at
 * @param max the maximum speed the bot could be travelling at
 * @param roundness how jerky the movement should be @note(the lower the roundness, the more jerky)
 * @param thickness ratio in 
 * 
 */
struct gainScheduleParams {
        float min = 0;
        float max = 127;
        float roundness = 1;
        float thickness = 3;
};

class Chassis {
    public:
        PID drivePID;
        PID turnPID;
        PID smallturnPID;
        PID swingPID;
        PID headingPID;
        PID backwardPID;
        PID arcPID;

        
        /**
         * @brief Struct for chassis
         * 
         * @param drivetrain drivetrain setup
         * @param pidsetup uses the pidSetup struct to setup pid constants for movements
         * 
         */
        Chassis(driveTrain drivetrain, pidSetup pidsetup);
        /**
         * @brief calibrate chassis
         */
        void calibrate();

        /**
         * @brief sets the odometry pose of the bot
         *
         * @param x1 set the x value
         * @param y1 set the y value
         * @param theta1 set the theta value(in degrees)
         */
        void setPose(float x1, float y1, float theta1);
        /**
         * @brief set the heading of the bot specifically- pain to always change points when you just have to change
         * heading.
         *
         * @param heading set heading in degrees
         */
        void setHeading(float heading);
        /**
         * @brief moves the bot using tank fashion(left controls leftside of the base, right controls rightside of the
         * base)
         *
         * @param left left power
         * @param right right power
         * @param curve 0<=curve; curve driver control stick values
         */
        void tank(float left, float right, float curve = 0);
        /**
         * @brief moves the bot using arcade fashion(lateral controls forward/backward movements while angular puts turn
         * bias on the powers)
         *
         * @param lateral left power
         * @param angular right power
         * @param curve 0<=curve; curve driver control stick values
         */
        void arcade(float lateral, float angular, float curve = 0);
        /**
         * @brief moves the bot forward using forwards or backwards PID
         *
         * @param distance distance the bot should move to(in inches)
         * @param maxSpeed the max speed the robot can travel in(out of 127).
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
         * @param heading specified heading the robot should travel at while approaching lateral target
         * 
         */
        void move(float distance, gainScheduleParams gainSched, float maxSpeed = 127, bool async = false);

        /**
         * @brief moves the bot forwards or backwards using the forward or backward PID and exits into the user
         * specified next movement. ANOTHER MOVEMENT AFTER THIS ONE IS REQUIRED.
         *
         * @param distance distance the bot should move to(in inches)
         * @param exitrange how much distance the robot should settle in(ie. want to move 24 inches: distance = 26,
         * exitrange = 2)
         * @param maxSpeed the max speed the robot can travel in(out of 127)
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
         * @param heading specified heading the robot should travel at while approaching lateral target
         * 
         */
        /*float distance, gainScheduleParams gainSched = {}, float exitrange, float timeout, float maxSpeed, bool async*/
        void moveWithoutSettle(float distance, gainScheduleParams gainSched, float exitRange, float timeout, float maxSpeed = 127, bool async = false);
        /**
         * @brief moves the bot forwards or backwards using the forward or backward PID and exits after a certain time
         * is reached
         *
         * @param distance distance the bot should move to(in inches)
         * @param timeout how much time the robot should move until
         * @param maxSpeed the max speed the robot can travel in(out of 127)
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
         * @param heading specified heading the robot should travel at while approaching lateral target
         * 
         */
        void moveWithoutSettleTime(float distance, gainScheduleParams gainSched, float timeout, float maxSpeed = 127, bool async = false);
        /**
         * @brief moves the bot to a certain point on the field
         *
         * @param x1 target x coordinate in inches
         * @param y1 target y coordinate in inches
         * @param timeout target y coordinate in inches
         * @param maxSpeed the max speed the robot should travel in(out of 127).
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
         */
        void moveToPoint(float x1, float y1, gainScheduleParams gainSched, int timeout, float maxSpeed = 127, bool async = false);
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
         * @param chasePower how fast the robot should go during the movement. If its higher, the accuracy of the
         * movement is less.
         * @param lead 0 < lead < 1 how much the movement should curve
         * @param smoothness 0 < smoothness < 1 how smooth the movement should be.
         * @param gLead weight for ghost point
         *
         */
        void moveToPose(float x1, float y1, float theta1, gainScheduleParams gainSched, float timeout, MoveToPoseParams params = {}, bool async = false);

        /**
         * @brief turns the robot to a target angle
         *
         * @param heading target angle
         * @param isSmallTurn is the angle relatively small?
         * @param maxSpeed max speed the robot should move at
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.

         *
        */
        void turn(float heading, bool isSmallTurn, float maxSpeed = 127, bool async = false);

        /**
         * @brief swings to a target angle with specific side of base
         *
         * @param heading target angle
         * @param isLeft true = moves to the target angle left, false = moves to the target angle right
         * @param maxSpeed max speed the robot should move at
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.

         *
        */
        void swing(float heading, bool isLeft, float maxSpeed = 127, bool async = false);
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
        void swingWithoutSettle(float heading, bool isLeft, float degreeRange, float maxSpeed = 127,
                                  bool async = false);
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
        void arc(float heading, double leftMult, double rightMult, float maxSpeed = 127, bool async = false);
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
        void arcNonSettle(float heading, double leftMult, double rightMult, float maxSpeed = 127, bool async = false);
        /**
         * @brief uses a preset radius to move the robot to a certain angle using radius(in inches)
         *
         * @param heading target angle
         * @param radius imaginary radius distance from a point that the robot is circling around
         * @param maxSpeed max speed the robot should move at
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.

         *
        */
        void radius_arc(float heading, float radius, float maxSpeed = 127, bool async = false);
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
        void turn_to_point(float x1, float y1, int timeout, bool forwards = true, float maxSpeed = 127,
                           bool async = false);
        /**
         * @brief follows path using ramsete(std::vector<Pose>{})
         *
         * @param pPath list of points the robot should follow
         * @param targetLinVel in RPM, how fast the robot should move linear wise.
         * @param targetAngVel in radians/sec, how fast the angular velocity the robot should have during turns or
         curves
         * @param timeOut how long the entire movement should take
         * @param errorRange how much distance the robot can ignore before moving on to the next point
         * @param beta

         *
        */

        void follow_path(std::vector<Pose> pPath, float targetLinVel, float targetAngVel, float timeOut,
                         float errorRange, float beta, float zeta, bool reversed);
        /**
         * @brief Wait until the robot has traveled a certain distance along the path
         *
         * @note Units are in inches if current motion is moveTo or follow, degrees if using turnTo
         *
         * @param dist the distance the robot needs to travel before returning
         */
        void waitUntil(float error);
        /**
         * @brief Wait until the robot has completed the path
         *
         */
        void waitUntilDone();
        /**
         * @brief Cancels the currently running motion.
         * If there is a queued motion, then that queued motion will run.
         */
        void cancelMotion();
        /**
         * @brief Cancels all motions, even those that are queued.
         * After this, the chassis will not be in motion.
         */
        void cancelAllMotions();
        /**
         * @return whether a motion is currently running
         */
        bool isInMotion() const;

        // odom values
        double x = 0;
        double y = 0;
        double heading = 0;

    protected:
        /**
         * @brief Indicates that this motion is queued and blocks current task until this motion reaches front of queue
         */
        void requestMotionStart();
        /**
         * @brief Dequeues this motion and permits queued task to run
         */
        void endMotion();

    private:
        const float defaultChasePower;
        void moveChassis(float linearVelocity, float angularVelocity);
        void ramsete(Pose targetPose, Pose currentPose, float targetAngularVelocity, float targetLinearVelocity,
                     float beta, float zeta);
        int FindClosest(Pose pose, std::vector<Pose> pPath, int prevCloseIndex = 0);
        pros::MotorGroup* leftMotors;
        pros::MotorGroup* rightMotors;
        pros::IMU* imu;
        float angleError(float angle1, float angle2, bool radians);
        const float wheelDiameter;
        const float trackWidth;
        const float gearRatio;
        bool motionRunning = false;
        bool motionQueued = false;
        pros::Mutex mutex;
        double distTravelled = 0;
        float get_absolute_heading();
};
}; // namespace gfrLib

#pragma once

#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include "pose.hpp"
#include "pid.hpp"
#include "util.hpp"
#include <functional>
#include "opcontrol.hpp"

namespace gfrLib{
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
        Chassis(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, pros::IMU* inertial,
                const float wheelDiameter, const float trackWidth, const float gearRatio, PID drivePID, PID backwardPID, PID turnPID,PID smallturnPID, PID swingPID, PID arcPID, PID headingPID);
        /**
         * @brief calibrate chassis
        */
        void calibrate();
        void odometry_loop();
        /**
         * @brief sets the odometry pose of the bot
         * 
         * @param x1 set the x value
         * @param y1 set the y value
         * @param theta1 set the theta value(in degrees)
        */
        void set_pose(float x1, float y1, float theta1);
        /**
         * @brief set the heading of the bot specifically- pain to always change points when you just have to change heading.
         * 
         * @param heading set heading in degrees
        */
        void set_heading(float heading);
        /**
         * @brief moves the bot using tank fashion(left controls leftside of the base, right controls rightside of the base)
         * 
         * @param left left power
         * @param right right power
        */
        void tank(float left, float right, float curve = 0);
        /**
         * @brief moves the bot using arcade fashion(lateral controls forward/backward movements while angular puts turn bias on the powers)
         * 
         * @param lateral left power
         * @param angular right power
        */
        void arcade(float lateral, float angular, float curve = 0);
        /**
         * @brief moves the bot forward using forwards or backwards PID
         * 
         * @param distance distance the bot should move to(in inches)
         * @param maxSpeed the max speed the robot should travel in(out of 127). 
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
        */
        void move(float distance, float maxSpeed=127, bool async = false);
    
        /**
         * @brief moves the bot forwards or backwards using the forward or backward PID and exits into the user specified next movement. ANOTHER MOVEMENT AFTER THIS ONE IS REQUIRED.
         * 
         * @param distance distance the bot should move to(in inches)
         * @param exitrange how much distance the robot should settle in(ie. want to move 24 inches: distance = 26, exitrange = 2)
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
        */
        void move_without_settle(float distance, float exitrange, float maxSpeed = 127,bool async = false);
        /**
         * @brief moves the bot forwards or backwards using the forward or backward PID and exits after a certain time is reached
         * 
         * @param distance distance the bot should move to(in inches)
         * @param timeout how much time the robot should move until 
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
        */
        void move_without_settletime(float distance, float timeout, float maxSpeed = 127, bool async = false); 
        /**
         * @brief moves the bot to a certain point on the field
         * 
         * @param x1 target x coordinate in inches
         * @param y1 target y coordinate in inches
         * @param timeout target y coordinate in inches
         * @param maxSpeed the max speed the robot should travel in(out of 127). 
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
        */
        void move_to_point(float x1, float y1, int timeout, float maxSpeed = 127,bool async = false);
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
        void move_to_pose(float x1, float y1, float theta1, int timeout, bool forwards, float maxSpeed, bool async,float chasePower,
                          float lead, float smoothness);

        /**
         * @brief turns the robot to a target angle
         * 
         * @param heading target angle
         * @param isSmallTurn is the angle relatively small?
         * @param maxSpeed max speed the robot should move at
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
         
         * 
        */
        void turn(float heading, bool isSmallTurn, float maxSpeed=127,bool async = false);
        
        /**
         * @brief swings to a target angle with specific side of base
         * 
         * @param heading target angle
         * @param isLeft true = moves to the target angle left, false = moves to the target angle right
         * @param maxSpeed max speed the robot should move at
         * @param async if selected, subsystem actions such as deploying pneumatics during the movement can occur.
         
         * 
        */
        void swing(float heading, bool isLeft, float maxSpeed=127,bool async = false);
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
        void swing_without_settle(float heading, bool isLeft, float degreeRange, float maxSpeed = 127, bool async = false);
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
        void arc(float heading, double leftMult, double rightMult, float maxSpeed = 127,bool async = false);
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
        void arc_non_settle(float heading, double leftMult, double rightMult, float maxSpeed = 127,bool async = false);
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
        void turn_to_point(float x1, float y1, int timeout, bool forwards = true,float maxSpeed=127,bool async = false);
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

        void follow_path(std::vector<Pose> pPath, float targetLinVel, float targetAngVel, float timeOut, float errorRange, float beta, float zeta, bool reversed);
        /**
         * @brief Wait until the robot has traveled a certain distance along the path
         *
         * @note Units are in inches if current motion is moveTo or follow, degrees if using turnTo
         *
         * @param dist the distance the robot needs to travel before returning
         */
        void wait_until(float error);
        /**
         * @brief Wait until the robot has completed the path
         *
         */
        void wait_until_done();
        /**
         * @brief Cancels the currently running motion.
         * If there is a queued motion, then that queued motion will run.
         */
        void cancel_motion();
        /**
         * @brief Cancels all motions, even those that are queued.
         * After this, the chassis will not be in motion.
         */
        void cancel_all_motions();
        /**
         * @return whether a motion is currently running
         */
        bool is_in_motion() const;
        
        //odom values
        double x = 0;
        double y = 0;
        double heading = 0;

        
    protected:
        /**
         * @brief Indicates that this motion is queued and blocks current task until this motion reaches front of queue
         */
        void request_motion_start();
        /**
         * @brief Dequeues this motion and permits queued task to run
         */
        void end_motion();

        
        
        
        
    private:
        void moveChassis(float linearVelocity, float angularVelocity);
        void ramsete(Pose targetPose, Pose currentPose, float targetAngularVelocity, float targetLinearVelocity, float beta, float zeta);
        int FindClosest(Pose pose, std::vector<Pose> pPath, int prevCloseIndex=0);
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
        
};};
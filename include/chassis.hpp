#pragma once

#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include "pose.hpp"
#include "pid.hpp"


class Chassis {
    public:
        
        PID drivePID;
        PID turnPID;
        PID smallturnPID;
        PID swingPID;
        PID headingPID;
        PID backwardPID;
        PID arcPID;
    

        Chassis(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, pros::IMU* inertial,
                const float wheelDiameter, const float trackWidth, const float gearRatio, PID drivePID, PID backwardPID, PID turnPID,PID smallturnPID, PID swingPID, PID arcPID, PID headingPID);

        void calibrate();
        void setPose(float x1, float y1, float theta1);
        
        void setHeading(float heading);
        void tank(float left, float right);
        void arcade(float lateral, float angular);
    
        void move(float distance, float maxSpeed=127, bool async = false);
        void turn(float heading, float maxSpeed=127,bool async = false);
        void turnsmall(float heading, float maxSpeed = 127,bool async = false);
        void swing(float heading, bool isLeft, float maxSpeed=127,bool async = false);
        void arc(float heading, double leftMult, double rightMult, float maxSpeed = 127,bool async = false);
        void arcnonsettle(float heading, double leftMult, double rightMult, float maxSpeed = 127,bool async = false);
        void move_without_settle(float distance, float exitrange,bool async = false);
        void swing_without_settle(float heading, bool isLeft, float timeout,bool async = false);
        
        
       void movewithheading(float distance, float heading, float maxSpeed = 127,bool async = false);
        
        void moveToPoint(float x1, float y1, int timeout, float maxSpeed = 127,bool async = false);
        void turnToPoint(float x1, float y1, int timeout, float maxSpeed=127,bool async = false);
        void move_without_settletime(float distance, float timeout,bool async = false); 
        void waitUntilDist(float dist);
        void moveToPose(float x1, float y1, float theta1, int timeout, bool forwards, float maxSpeed, bool async,float chasePower,
                          float lead, float smoothness, bool linearexit, float linearexitrange);

        double x = 0;
        double y = 0;
        double heading = 0;
        
        
        void followPath(std::vector<Pose> pPath, float targetLinVel, float targetAngVel, float timeOut, float errorRange, float beta, float zeta, bool reversed);
        
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
        
        pros::Mutex mutex;
        
        double distTravelled = 0;
        float get_absolute_heading();
};

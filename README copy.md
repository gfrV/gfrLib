# Gael Force Robotics Lib


## Credit
Gael Force V- 5327V

## Installation

> [!NOTE]
> Note that PROS is only support on ancient compiler

1. Download zip.
2. Open PROS terminal after unzipping the folder.
3. Run `pros mu` in the PROS integrated terminal to run the code!

## Additional Versions

Want a different version of our library? Let us know by opening an issue [here](https://github.com/AgilanSam/5327V/issues/new)
P.S adding async actions to all movements soon

## Setup
```
pros::IMU inertial(12); //inertial sensor

//left motors
pros::MotorGroup leftMotors(std::initializer_list<pros::Motor> {
    pros::Motor(-15, pros::E_MOTOR_GEAR_BLUE), // left front motor
    pros::Motor(-11, pros::E_MOTOR_GEAR_BLUE), // left middle motor
    pros::Motor(-14, pros::E_MOTOR_GEAR_BLUE), //left back motor
});

//right motors
pros::MotorGroup rightMotors(std::initializer_list<pros::Motor> {
    pros::Motor(16, pros::E_MOTOR_GEAR_BLUE), // right front motor
    pros::Motor(17, pros::E_MOTOR_GEAR_BLUE), // right middle motor
    pros::Motor(18, pros::E_MOTOR_GEAR_BLUE), //right back motor
});


//setup the chassis object
Chassis chassis(
    //left motors, right motors, inertial sensor
    &leftMotors, &rightMotors, &inertial, 
    //wheel diameter, gear ratio
    2.75, 1, 
    //forward PID
    PID(10, 0, 45, 0, 0.75, 250, 2, 750, 7000),
    //backward PID
    PID(10, 0, 45, 0, 0.75, 250, 1, 750, 7000),
    //turnPID
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
```
> customizable forward and backward PID's due to the variablity of the robot's weight.
> Heading PID to ensure the robot travels at its intended angle in every movement.

## Perks
 > Multiple PID movements
 > Odometry & Odometry-Based movements

## PID struct 
> Use a PID controller in various robot applications such as lift, etc.

PID(kP, kI, kD, kImax, settleError, settleTime, maxSettleError, maxSettleTime, maxTime)
- Kp(proportional)
- kI(integral)
- kD(derivative)
- settleError(small settle error)
- settleTime(for small error)
- maxSettleError(for long distances)
- maxSettleTime(longest time it takes to settle)
- maxTime(how long the entire movement)
  
## Odometry
 > IME Odometry(adding Kalman Filters soon)
```
//sets the coordinates and angle for the odometry system
chassis.setPose(x,y,angle);
```
```
//gets the odometry coordinates and angle
chassis.getPose(); //returns a std::pair with x,and y. To get the individual x, y, and angle use:
chassis.x, chassis.y, chassis.heading
```


## PID Movement Functions
### chassis.move(distance, maxSpeed = 127);
 ```
chassis.move(24);
chassis.move(-24);

```
### chassis.turn(angle, maxSpeed = 127);
 ```
chassis.turn(90);
chassis.turn(-90);

```
### chassis.swing(angle, bool isLeft,maxSpeed = 127);
 ```
chassis.swing(90, false); // swings 90 to the right
chassis.move(-90, true); //swings -90 to the left

```
### chassis.arc(angle, leftMult, rightMult);
 ```
// go to an angle with a certain curvature
chassis.arc(90, 0.3, 0.6);
```
## Odometry Movement Functions

### chassis.moveToPoint(x,y,timeout, maxSpeed);
 ```
// moves to a point after predetermining the angle and distance to the target point from the odom pose
chassis.moveToPoint(24, 24, 2000, 127);
```
### chassis.activeMoveToPoint(x,y,timeout, maxSpeed); //recomended
 ```
// moves to a point while constantly updating the distance and angular components to reach the certain point.
chassis.activeMoveToPoint(24, 24, 2000, 127);
```
### chassis.moveToPose(x, y, angle, timeout, bool forwards,maxSpeed,async, chasePower, lead, bool linearexit, float linearexitrange, float smoothness);
 ```
// moves to a target pose from the odom pose with customizable parameters such as its chasepower, curvature, maxspeed, smoothness, and the ability to chain into the next movement. This is going to get added in the next update.
chassis.moveToPose(24, 24, 90, 2000, true, 127, 50, 0.6, false, 0, 1);
// smoothness: higher = smoother since it prioritizes linear output more. 
```

## Driver Control Features

### chassis.tank(controller input, controller input) can be used to control the robot with a tank fashion.
### chassis.arcade(controller input, controller input) can be used to control the robot with an arcade fashion.


#### _Contributions welcome. Anything missing? Send in a pull request. Thanks._
Follow  ([@gaelforcev](https://instagram.com/gaelforcev)) on Instagram for updates.

---

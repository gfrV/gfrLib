#include "gfrLib/chassis.hpp"

using namespace gfrLib;

int Chassis::FindClosest(Pose pose, std::vector<Pose> pPath, int prevCloseIndex) {
    // Find the closest point to the robot
    int closeIndex = 0;
    float minDistance = INT_MAX;
    for (int i = prevCloseIndex; i < pPath.size(); i++) {
        float dist = pose.distance(pPath.at(i));
        if (dist < minDistance) {
            closeIndex = i;
            minDistance = dist;
        }
    }
    return closeIndex;
}


void Chassis::moveChassis(float linearVelocity, float angularVelocity) {
    // compute left and right velocities
    float leftVelocity = (2 * linearVelocity - angularVelocity * trackWidth) / 2; // inches/sec
    float rightVelocity = (2 * linearVelocity + angularVelocity * trackWidth) / 2; // inches/sec

    // calculate left and right wheel rpm
    float leftRPM = leftVelocity * 60.0 / (wheelDiameter * M_PI); // rpm
    float rightRPM = rightVelocity * 60.0 / (wheelDiameter * M_PI); // rpm

    // calculate the left and right motor rpm
    float leftMotorRPM = leftRPM * (gearRatio); // gearset
    float rightMotorRPM = rightRPM * (gearRatio); // gearset

    // move chassis
    leftMotors->move_velocity(leftMotorRPM);
    rightMotors->move_velocity(rightMotorRPM);
}

void Chassis::ramsete(Pose targetPose, Pose currentPose, float targetAngularVelocity, float targetLinearVelocity,
                      float beta, float zeta) {
    // compute global error
    Eigen::MatrixXd globalError(1, 3);
    globalError << targetPose.x - currentPose.x, targetPose.y - currentPose.y,
        degToRad(targetPose.theta) - currentPose.theta;

    // compute transformation matrix
    Eigen::MatrixXd transformationMatrix(3, 3);
    transformationMatrix << cos(currentPose.theta), sin(currentPose.theta), 0, -sin(currentPose.theta),
        cos(currentPose.theta), 0, 0, 0, 1;

    // compute local error
    Eigen::MatrixXd localError = globalError * transformationMatrix;
    // compute k gain
    float k =
        2 * zeta *
        std::sqrt(targetAngularVelocity * targetAngularVelocity + beta + targetLinearVelocity * targetLinearVelocity);
    // compute angular velocity
    float angularVelocity = targetAngularVelocity * cos(localError(0, 2)) + k * localError(0, 0);
    // compute linear velocity
    float linearVelocity = targetLinearVelocity + k * localError(0, 2) +
                           (beta * linearVelocity * sin(localError(0, 2)) * localError(0, 1) / localError(0, 2));

    // move chassis
    moveChassis(linearVelocity, angularVelocity);
} // works


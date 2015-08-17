//
// Created by robin on 11-8-15.
//

#include "SquareSetpointGenerator.h"

SquareSetpointGenerator::SquareSetpointGenerator(double cx, double cy, double height, double _size) {
    geometry_msgs::Point p;
    p.x = cx;
    p.y = cy;
    p.z = height;
    centerPoint = p;
    squareSize = _size;

    currentState = 0;
}

int SquareSetpointGenerator::getSteps() {
    return 4;
}

void SquareSetpointGenerator::setStep(int step) {
    currentState = step;
}

geometry_msgs::Pose SquareSetpointGenerator::getNextSetpoint() {
    float dx = .5 * squareSize;
    float dy = .5 * squareSize;
    switch (currentState) {
        case 0:
            dx = -dx;
            break;
        case 1:
            break;
        case 2:
            dy = -dy;
            break;
        case 3:
            dx = -dx;
            dy = -dy;
    }

    geometry_msgs::Pose publishPose;
    publishPose.position.x = centerPoint.x + dx;
    publishPose.position.y = centerPoint.y + dy;
    publishPose.position.z = centerPoint.z;

    publishPose.orientation.x = 0;
    publishPose.orientation.y = 0;
    publishPose.orientation.z = 1;
    publishPose.orientation.w = 0;

    currentState = (currentState + 1) % (getSteps());

    return publishPose;
}
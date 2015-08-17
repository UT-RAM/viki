//
// Created by robin on 11-8-15.
//

#ifndef PROJECT_SQUARESETPOINTGENERATOR_H
#define PROJECT_SQUARESETPOINTGENERATOR_H

#include "AbstractSetpointGenerator.h"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

class SquareSetpointGenerator : public AbstractSetpointGenerator {
private:
    int currentState;
    geometry_msgs::Point centerPoint;
    double squareSize;

public:
    SquareSetpointGenerator(double cx, double cy, double height, double size);
    geometry_msgs::Pose getNextSetpoint();
    int getSteps();
    void setStep(int step);
};

#endif //PROJECT_SQUARESETPOINTGENERATOR_H


//
// Created by robin on 11-8-15.
//

#ifndef PROJECT_ABSTRACTSETPOINTGENERATOR_H
#define PROJECT_ABSTRACTSETPOINTGENERATOR_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include "SetpointPublisher.h"

class SetpointPublisher;
class AbstractSetpointGenerator {
protected:
    SetpointPublisher* publisher;
public:
//    geometry_msgs::Point getNextSetpoint() ;
};


#endif //PROJECT_ABSTRACTSETPOINTGENERATOR_H

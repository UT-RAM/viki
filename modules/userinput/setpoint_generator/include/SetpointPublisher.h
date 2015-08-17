//
// Created by robin on 11-8-15.
//

#ifndef PROJECT_SETPOINTPUBLISHER_H
#define PROJECT_SETPOINTPUBLISHER_H

#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include "SquareSetpointGenerator.h"
#include "AbstractSetpointGenerator.h"

class SquareSetpointGenerator;
class SetpointPublisher {
private:
    ros::Publisher setpoint_pub;
    ros::NodeHandle* nh;
    ros::Timer pub_timer;

    SquareSetpointGenerator* setpointGenerator;
public:
    SetpointPublisher();
    void publishSetpoint(const ros::TimerEvent& event);
};


#endif //PROJECT_SETPOINTPUBLISHER_H

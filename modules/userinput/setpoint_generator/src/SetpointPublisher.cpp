//
// Created by robin on 11-8-15.
//

#include "SetpointPublisher.h"

SetpointPublisher::SetpointPublisher()
{
    nh = new ros::NodeHandle("~");

    // create publisher
    setpoint_pub = nh->advertise<geometry_msgs::Pose>("viki_setpoint", 5);

    // read parameters
    double period_time, period_phase_shift_time;
    if (!nh->getParam("period_time", period_time)) {
        ROS_WARN("Period time parameter not well read!");
    }
    nh->getParam("period_phase_shift_time", period_phase_shift_time);

    setpointGenerator = new SquareSetpointGenerator(0.0, 0.0, 1, 1.0);

    // Calculate the rate of publishing
    int steps = setpointGenerator->getSteps();
    double publish_frequency = (period_time / steps);
    int step_initial_offset = (int) ((period_phase_shift_time / period_time) * steps);

    std::cout << "Publish frequency:" << publish_frequency << "\n";

    setpointGenerator->setStep(step_initial_offset);
    pub_timer = nh->createTimer(ros::Duration(publish_frequency), &SetpointPublisher::publishSetpoint, this);
}

void SetpointPublisher::publishSetpoint(const ros::TimerEvent& event) {
    setpoint_pub.publish(setpointGenerator->getNextSetpoint());
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "setpoint_generator");
    SetpointPublisher sp;

    ros::spin();
    return 0;
}
// 23/07/2014, Alex Kamphuis, a.kamphuis-1@student.utwente.nl, RAM, UTwente
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

#include <stdlib.h>
#include <time.h>

/* This node setpoints on a cirlce at /setpoint */


int main(int argc, char **argv)
{
	ros::init(argc, argv, "dummy_setpoint"); // init rosnode
	ROS_INFO("initialized circe dummy setpoint publisher");
	/*set verbosity levels to debug */
		/*
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
		{
	   		ros::console::notifyLoggerLevelsChanged();
		}
		*/
	ros::NodeHandle n; // init NodeHandle
	
	/* create publishers */
		ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("setpoint",100);
		ROS_DEBUG("advertised topic");

	/* parameters */
		int frequency = 	10;	
		float loop_time = 	10; // time for a full cirlce
		float r 		= 	5;
		float theta		=	0; // start angle
		ROS_DEBUG("parameters set");

	/* one time calculations: */
		ros::Rate loop_rate(frequency);
		float nsteps = frequency*loop_time;

	/* loop to send new data at the previously defined frequency*/
	while (ros::ok())
	{
		ROS_DEBUG("generating message...");
		geometry_msgs::Pose msg; // create a pose message
		
		/* define random positions */
		msg.position.x = r * sin(theta);
		msg.position.y = r * cos(theta);
		msg.position.z = 1;

		/* define orientation */
		msg.orientation.x = 0;
		msg.orientation.y = 0;
		msg.orientation.z = 1;
		msg.orientation.w = 0;

		pose_pub.publish(msg);
		ROS_DEBUG("message published");
		ros::spinOnce();
		ROS_DEBUG("spinOnce conducted");

		/* update theta */
		theta = theta+(2*3.1415)/nsteps;
		ROS_DEBUG("theta updated, going to sleep");
		ROS_INFO_ONCE("Publishing setpoints in a cirlce");
		loop_rate.sleep();
	}
return 0;
}
// 23/07/2014, Alex Kamphuis, a.kamphuis-1@student.utwente.nl, RAM, UTwente
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

/* This node publishes the pose of an static object, so you can test without having to place an obstacle*/


int main(int argc, char **argv)
{
	ros::init(argc, argv, "dummy_publisher"); // init rosnode
	ros::NodeHandle n; // init NodeHandle

	/* set radius and dynamic properties in parameter server */
	srand(time(NULL));
	n.setParam("radius",0.5);
	//n.setParam("vmax",5);
	//n.setParam("amax",10);

	/* create publisher */
		ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("unfiltered_pose",100);
		
	
	/* set loop rate (frequency) */
		ros::Rate loop_rate(100);

	/* loop to send new data at the previously defined frequency*/
	//int count=0; can be used to sen unique messages
		while (ros::ok())
		{
			geometry_msgs::Pose msg; // create a pose message
						
			/* define positions */
			msg.position.x = 1;
			msg.position.y = 1;
			msg.position.z = 1;

			/* define orientation */
			msg.orientation.x = 0;
			msg.orientation.y = 0;
			msg.orientation.z = 0;
			msg.orientation.w = 1;

			pose_pub.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();
			//++count; // can be used to send unique messages
		}
	return 0;
}
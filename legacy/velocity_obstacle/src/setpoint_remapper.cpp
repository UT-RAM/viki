#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

/*
	This function remaps a setpoint to the /setpoint topic
*/

class Remap
{
private:
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Publisher pub;
public:
	Remap()
	{
		sub = n.subscribe("input",1,&Remap::setpointCallback,this);
		pub = n.advertise<geometry_msgs::Pose>("/setpoint",1);

		ROS_INFO_STREAM("Remapping fromo: "<<sub.getTopic()<<" to: /setpoin");
	}

	~Remap()
	{
		ROS_WARN("The remapper has stopped");
	}

	void setpointCallback(const geometry_msgs::Pose setpoint)
	{
		ROS_DEBUG("Got setpoint message that I need to redirect");
		pub.publish(setpoint);
		ROS_DEBUG("Published setpoint to /setpoint topic");
	}
};




int main(int argc, char** argv)
{
	// initialize ros
	ros::init(argc, argv, "setpoint_remapper");
	
	/* set verbosity level */
		
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
				{
			   		ros::console::notifyLoggerLevelsChanged();
				}
		

	// make the class
		ROS_INFO_STREAM("starting the remapper..");
		Remap remap;
		
	// spin
		ros::spin();
		ROS_INFO_STREAM("the setpoint remapper has stopped.");
	return 0;
}
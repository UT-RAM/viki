#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelStates.h>

/*
	This function remaps a model state from gazebo to a robotname/unfiltered pose so the master divisor can use it
*/

class Remap
{
private:
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Publisher pub;
	int modelno_;
public:
	Remap(int modelno)
	{
		modelno_=modelno;
		sub = n.subscribe("gazebo/model_states",1,&Remap::stateCallback,this);
		pub = n.advertise<geometry_msgs::Pose>("output",1);

		ROS_INFO_STREAM("Remapping modelno: "<<modelno<<" to: "<<pub.getTopic());
	}

	~Remap()
	{
		ROS_WARN("The remapper has stopped");
	}

	void stateCallback(const gazebo_msgs::ModelStates msg)
	{
		ROS_DEBUG("Got a gazebo state message");
		geometry_msgs::Pose posemsg;
		posemsg.position.x=msg.pose[modelno_].position.x;
		posemsg.position.y=msg.pose[modelno_].position.y;
		posemsg.position.z=msg.pose[modelno_].position.z;

		/* orientation is unused */
		posemsg.orientation.x=0;
		posemsg.orientation.y=0;
		posemsg.orientation.z=0;
		posemsg.orientation.w=0;

		pub.publish(posemsg);
		ROS_DEBUG("Published pose to output topic");
	}
};




int main(int argc, char** argv)
{
	// initialize ros
	ros::init(argc, argv, "pose_remapper");
	ros::NodeHandle nh("~");
	int modelno;
	if(nh.getParam("modelno",modelno))
	{
		ROS_DEBUG("got modelno from server");
	}
	else
	{
		modelno = 11;
		ROS_WARN("Unable to get modelno parameter from server, resuming as if it was 11");
	}

	/* set verbosity level */
		/*
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
				{
			   		ros::console::notifyLoggerLevelsChanged();
				}
		*/

	// make the class
		Remap remap(modelno);
	// spin
		ros::spin();
	return 0;
}
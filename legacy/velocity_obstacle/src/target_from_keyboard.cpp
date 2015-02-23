#include <ros/ros.h>
#include <geometry_msgs/Point.h>
using std::cin;

/*
	this function accepts input from the keyboard to simply publish a target message.
*/

int main(int argc, char **argv)
{
	ros::init(argc,argv,"target_from_keyboard");

	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Point>("target",1);

	float x;
	float y;
	float z;
	geometry_msgs::Point	msg;
	while(ros::ok())
	{
		ROS_INFO_STREAM("key in a new setpoint x-value");
		cin>>x;
		if(!cin || cin.fail())
		{
    		ROS_ERROR("Invalid");
    		cin.clear();
    		cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    		continue;
		}
		else
		{
		    ROS_INFO_STREAM("valid");
		}
		
		ROS_INFO_STREAM("key in a new setpoint y-value");
		cin>>y;
		if(!cin || cin.fail())
		{
    		ROS_ERROR("Invalid");
    		cin.clear();
    		cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    		continue;
		}
		else
		{
		    ROS_INFO_STREAM("valid");
		}


		ROS_INFO_STREAM("key in a new setpoint z-value");
		cin>>z;
		if(!cin || cin.fail())
		{
    		ROS_ERROR("Invalid");
    		cin.clear();
    		cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    		continue;
		}
		else
		{
		    ROS_INFO_STREAM("valid");
		}

		ROS_INFO_STREAM("you keyed in: "<<x<<","<<y<<","<<z);

		msg.x=x;
		msg.y=y;
		msg.z=z;

		pub.publish(msg);

		ROS_INFO_STREAM("published new target");
	}
	

}
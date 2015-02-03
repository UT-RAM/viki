#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class Remap
{
private:
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Publisher pub;
public:
	Remap()
	{
		sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel",10,&Remap::cmdvelCallback,this);
		pub = n.advertise<geometry_msgs::Twist>("/robotname/cmd_vel",10);
	}

	~Remap()
	{

	}

	void cmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg)
	{
		pub.publish(*msg);
	}
};




int main(int argc, char** argv)
{
	// initialize ros
	ros::init(argc, argv, "cmd_vel_remapper");

	// make the class
	Remap remap;
	// spin
	ros::spin();
	return 0;
}
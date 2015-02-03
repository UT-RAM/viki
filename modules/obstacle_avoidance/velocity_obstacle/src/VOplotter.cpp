#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <ram_vo/VO_summary.h>
#include <geometry_msgs/Point.h>

void summaryCallback(const ram_vo::VO_summary::ConstPtr & summarymsg)
{
	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	const int ROBOT     = 0;
	const int OBJECT    = 1;
	const int TPLINE_1  = 2;
	const int TPLINE_2  = 3;
	const int SPHSHELL  = 4;
	const int TARGET    = 5;
	const int REQPOS    = 6;
	const int DESVEL    = 7;
	const int REQVEL    = 8;
	const float LINE_THICKNESS = 0.05;
	ROS_INFO("calback running");
	ram_vo::VO_summary summary = *summarymsg;

	// adjusting radii to confirm with the plotting tool, which want diameters in stead of radii
		summary.ra*=2;
		summary.rc*=2;	

	/* shape for robot */
		visualization_msgs::Marker robot;
		robot.header.frame_id="/my_frame";
		robot.header.stamp=ros::Time::now();

		robot.ns="Robot";
		robot.id=ROBOT;

		robot.type=visualization_msgs::Marker::SPHERE;
		robot.action=visualization_msgs::Marker::ADD;

		robot.pose.position.x=summary.a.x;
		robot.pose.position.y=summary.a.y;
		robot.pose.position.z=summary.a.z;
		robot.pose.orientation.x=0.0;
		robot.pose.orientation.y=0.0;
		robot.pose.orientation.z=0.0;
		robot.pose.orientation.w=1.0;

		robot.scale.x=summary.ra;
		robot.scale.y=summary.ra;
		robot.scale.z=summary.ra;

		robot.lifetime=ros::Duration();

		robot.color.r=0.0f;
		robot.color.g=1.0f;
		robot.color.b=0.0f;
		robot.color.a=0.5;

		marker_pub.publish(robot);
		ROS_DEBUG("published a new marker for the robot");

	/* marker for object */
		visualization_msgs::Marker object;
		object.header.frame_id="/my_frame";
		object.header.stamp=ros::Time::now();

		object.ns="Object";
		object.id=OBJECT;

		object.type=visualization_msgs::Marker::SPHERE;
		object.action=visualization_msgs::Marker::ADD;

		object.pose.position.x=summary.c.x;
		object.pose.position.y=summary.c.y;
		object.pose.position.z=summary.c.z;
		object.pose.orientation.x=0.0;
		object.pose.orientation.y=0.0;
		object.pose.orientation.z=0.0;
		object.pose.orientation.w=1.0;

		object.scale.x=summary.rc;
		object.scale.y=summary.rc;
		object.scale.z=summary.rc;

		object.lifetime=ros::Duration();

		object.color.r=1.0f;
		object.color.g=0.0f;
		object.color.b=0.0f;
		object.color.a=1;

		marker_pub.publish(object);
		ROS_DEBUG("published a new marker for the object");

	/* sphere for enhanced radius */
		visualization_msgs::Marker sphere_shell;
		sphere_shell.header.frame_id="/my_frame";
		sphere_shell.header.stamp=ros::Time::now();

		sphere_shell.ns="Shell";
		sphere_shell.id=SPHSHELL;

		sphere_shell.type=visualization_msgs::Marker::SPHERE;
		sphere_shell.action=visualization_msgs::Marker::ADD;

		sphere_shell.pose.position.x=summary.c.x;
		sphere_shell.pose.position.y=summary.c.y;
		sphere_shell.pose.position.z=summary.c.z;
		sphere_shell.pose.orientation.x=0.0;
		sphere_shell.pose.orientation.y=0.0;
		sphere_shell.pose.orientation.z=0.0;
		sphere_shell.pose.orientation.w=1.0;

		sphere_shell.scale.x=summary.ra+summary.rc;
		sphere_shell.scale.y=summary.ra+summary.rc;
		sphere_shell.scale.z=summary.ra+summary.rc;

		sphere_shell.lifetime=ros::Duration();

		sphere_shell.color.r=1.0f;
		sphere_shell.color.g=0.0f;
		sphere_shell.color.b=0.0f;
		sphere_shell.color.a=0.3;

		marker_pub.publish(sphere_shell);
		ROS_DEBUG("published a new marker the shell");

	/* marker for tangent line 1*/
		visualization_msgs::Marker line;
		line.header.frame_id="/my_frame";
		line.header.stamp=ros::Time::now();

		line.ns="VelocityObstacle";
		line.id=TPLINE_1;

		line.type=visualization_msgs::Marker::LINE_STRIP;
		line.action=visualization_msgs::Marker::ADD;

		geometry_msgs::Point p;
		p.x=summary.apex.x;
		p.y=summary.apex.y;
		p.z=summary.apex.z;
		line.points.push_back(p);

		p.x=summary.tp1.x;
		p.y=summary.tp1.y;
		p.z=summary.tp1.z;
		line.points.push_back(p);

		line.pose.position.x=0;
		line.pose.position.y=0;
		line.pose.position.z=0;
		line.pose.orientation.x=0;
		line.pose.orientation.y=0;
		line.pose.orientation.z=0;
		line.pose.orientation.w=1;

		line.scale.x=LINE_THICKNESS;

		line.lifetime=ros::Duration();

		line.color.r=0.0f;
		line.color.g=0.0f;
		line.color.b=0.0f;
		line.color.a=1;

		marker_pub.publish(line);
		ROS_DEBUG("published a new marker for the first tangent line");

	/* marker for tangent line 2 */
		visualization_msgs::Marker line2;
		line2.header.frame_id="/my_frame";
		line2.header.stamp=ros::Time::now();

		line2.ns="VelocityObstacle";
		line2.id=TPLINE_2;

		line2.type=visualization_msgs::Marker::LINE_STRIP;
		line2.action=visualization_msgs::Marker::ADD;

		//geometry_msgs::Point p;
		p.x=summary.apex.x;
		p.y=summary.apex.y;
		p.z=summary.apex.z;
		line2.points.push_back(p);

		p.x=summary.tp2.x;
		p.y=summary.tp2.y;
		p.z=summary.tp2.z;
		line2.points.push_back(p);

		line2.pose.position.x=0;
		line2.pose.position.y=0;
		line2.pose.position.z=0;
		line2.pose.orientation.x=0;
		line2.pose.orientation.y=0;
		line2.pose.orientation.z=0;
		line2.pose.orientation.w=1;

		line2.scale.x=LINE_THICKNESS;

		line2.lifetime=ros::Duration();

		line2.color.r=0.0f;
		line2.color.g=0.0f;
		line2.color.b=0.0f;
		line2.color.a=1;

		marker_pub.publish(line2);
		ROS_DEBUG("published a new marker for the second tangent line");

	/* target */
		visualization_msgs::Marker target;
		target.header.frame_id="/my_frame";
		target.header.stamp=ros::Time::now();

		target.ns="Target";
		target.id=TARGET;

		target.type=visualization_msgs::Marker::POINTS;
		target.action=visualization_msgs::Marker::ADD;

		target.pose.position.x=0;
		target.pose.position.y=0;
		target.pose.position.z=0;
		target.pose.orientation.x=0;
		target.pose.orientation.y=0;
		target.pose.orientation.z=0;
		target.pose.orientation.w=1;

		p.x=summary.target.x;
		p.y=summary.target.y;
		p.z=summary.target.z;
		target.points.push_back(p);

		target.scale.x=0.1;
		target.scale.y=0.1;

		target.lifetime=ros::Duration();

		target.color.r=1.0f;
		target.color.g=1.0f;
		target.color.b=0.0f;
		target.color.a=1;

		marker_pub.publish(target);
		ROS_DEBUG("published a new marker for the target");

	/* requested position */
		visualization_msgs::Marker req_pos;
		req_pos.header.frame_id="/my_frame";
		req_pos.header.stamp=ros::Time::now();

		req_pos.ns="Requested Position";
		req_pos.id=REQPOS;

		req_pos.type=visualization_msgs::Marker::POINTS;
		req_pos.action=visualization_msgs::Marker::ADD;

		req_pos.pose.position.x=0;
		req_pos.pose.position.y=0;
		req_pos.pose.position.z=0;
		req_pos.pose.orientation.x=0;
		req_pos.pose.orientation.y=0;
		req_pos.pose.orientation.z=0;
		req_pos.pose.orientation.w=1;

		p.x=summary.requested_position.x;
		p.y=summary.requested_position.y;
		p.z=summary.requested_position.z;
		req_pos.points.push_back(p);

		req_pos.scale.x=0.1;
		req_pos.scale.y=0.1;

		req_pos.lifetime=ros::Duration();

		req_pos.color.r=1.0f;
		req_pos.color.g=0.0f;
		req_pos.color.b=0.5f;
		req_pos.color.a=1;

		marker_pub.publish(req_pos);
		ROS_DEBUG("published a new marker for the req_pos");

	/* desired velocity (velocity pointing towards target*/
		visualization_msgs::Marker des_vel;
		des_vel.header.frame_id="/my_frame";

		des_vel.ns="Desired velocity";
		des_vel.id=DESVEL;

		des_vel.type=visualization_msgs::Marker::LINE_STRIP;
		des_vel.action=visualization_msgs::Marker::ADD;

		des_vel.pose.position.x=0;
		des_vel.pose.position.y=0;
		des_vel.pose.position.z=0;
		des_vel.pose.orientation.x=0;
		des_vel.pose.orientation.y=0;
		des_vel.pose.orientation.z=0;
		des_vel.pose.orientation.w=1;

		des_vel.scale.x=LINE_THICKNESS;

		p.x=summary.a.x;
		p.y=summary.a.y;
		p.z=summary.a.z;
		des_vel.points.push_back(p);

		p.x=summary.desired_velocity.x+summary.a.x;
		p.y=summary.desired_velocity.y+summary.a.y;
		p.z=summary.desired_velocity.z+summary.a.z;
		des_vel.points.push_back(p);

		des_vel.color.r=0.0f;
		des_vel.color.g=1.0f;
		des_vel.color.b=1.0f;
		des_vel.color.a=1;

		marker_pub.publish(des_vel);
		ROS_DEBUG("published a new marker for desired velocity");

	/* requested velocity */
		visualization_msgs::Marker req_vel;
		req_vel.ns="Requested velocity";
		req_vel.id=REQVEL;

		req_vel.type=visualization_msgs::Marker::LINE_STRIP;
		req_vel.action=visualization_msgs::Marker::ADD;

		req_vel.pose.position.x=0;
		req_vel.pose.position.y=0;
		req_vel.pose.position.z=0;
		req_vel.pose.orientation.x=0;
		req_vel.pose.orientation.y=0;
		req_vel.pose.orientation.z=0;
		req_vel.pose.orientation.w=1;

		req_vel.scale.x=LINE_THICKNESS;

		p.x=summary.a.x;
		p.y=summary.a.y;
		p.z=summary.a.z;
		req_vel.points.push_back(p);

		p.x=summary.a.x+summary.requested_velocity.x;
		p.y=summary.a.y+summary.requested_velocity.y;
		p.z=summary.a.z+summary.requested_velocity.z;
		req_vel.points.push_back(p);

		req_vel.color.r=0.0f;
		req_vel.color.g=0.5f;
		req_vel.color.b=0.5f;
		req_vel.color.a=1;

		marker_pub.publish(req_vel);
		ROS_DEBUG("published a new marker for requested velocity");
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber summary_sub = n.subscribe("/Lindsey/VO_summary",1, summaryCallback);

  ros::spin();
}
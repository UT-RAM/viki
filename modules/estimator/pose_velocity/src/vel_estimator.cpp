/**
  * Class VelEstimator
  *
  * Subscribes to a pose and estimates the velocity from that
  * Adapted from the State Estimation from the ram_ba_package
  * made by Cees Trouwborst
  *
  * Robin Hoogervorst
  */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

class VelEstimator {
private:
	ros::NodeHandle nh;

	ros::Subscriber pose_subscriber_;
	ros::Publisher odom_publisher_;

	std::vector <geometry_msgs::PoseStamped> poseBuffer; 	// Previous poses
	int pose_memory_; 							// Max poses stored	
	double uncertainty_band_; 					// Max band around pose to find line fitting

public:

	VelEstimator() {
		ros::NodeHandle nh("~");

		pose_memory_ = 15;
		nh.getParam("pose_memory", pose_memory_);

		uncertainty_band_ = .1;	
		nh.getParam("uncertainty_band", uncertainty_band_);

		pose_subscriber_ = nh.subscribe("/pose2odom/pose", 1, &VelEstimator::poseCallback, this);
		odom_publisher_ = nh.advertise<nav_msgs::Odometry>("/pose2odom/odom_output", 1);
	}

	void poseCallback(const geometry_msgs::PoseStamped pose_msg)
	{
		// add the pose to the buffer
		poseBuffer.push_back(pose_msg);

		// exit when not enough samples
		if (poseBuffer.size() < 0) return ;

		// decrease buffer if it's too large
		if(poseBuffer.size() > pose_memory_) {
			poseBuffer.erase(poseBuffer.begin());
		}

		int samplesBack = getFOAWSamplesBack();

		// initialise the poses and times to use
		int n = poseBuffer.size();
		geometry_msgs::Point p1 = poseBuffer[n-1].pose.position; //n-1 pose
		geometry_msgs::Point p2 = poseBuffer[n-1-samplesBack+1].pose.position;
		double t1 = poseBuffer[n-1].header.stamp.toSec();
		double t2 = poseBuffer[n-1-samplesBack+1].header.stamp.toSec();

		geometry_msgs::Twist vel;
		vel.linear.x = (p1.x - p2.x) / (t1 - t2);
		vel.linear.y = (p1.y - p2.y) / (t1 - t2);
		vel.linear.z = (p1.z - p2.z) / (t1 - t2);

		publishOdom(poseBuffer[n-1].header.stamp, pose_msg.pose, vel);
	}


	int getFOAWSamplesBack() {
		int n = poseBuffer.size();

		// In this loop, we check if, given a certain beginning of the window, all points fit the line.
		// Loops over a set of possible steps back.
		int samplesBack;
		bool optimalSampleReached;
		double windowSpeed_x, windowSpeed_y, windowSpeed_z;
		optimalSampleReached = false; 
		samplesBack = 1;
		while(!optimalSampleReached)
		{
			// initialise the poses and times to use
			geometry_msgs::Point p1 = poseBuffer[n-1].pose.position; //n-1 pose
			geometry_msgs::Point p2 = poseBuffer[n-1-samplesBack].pose.position;
			double t1 = poseBuffer[n-1].header.stamp.toSec();
			double t2 = poseBuffer[n-1-samplesBack].header.stamp.toSec();

			windowSpeed_x = (p1.x - p2.x)/(t1 - t2);
			windowSpeed_y = (p1.y - p2.y)/(t1 - t2);
			windowSpeed_z = (p1.z - p2.z)/(t1 - t2);

			// At this point, we have a line through the two poins we know (at last and last - steps back). 
			// We do not have to check those. Check all points inbetween
			bool allInBand;
			int i;
			allInBand = true;
			for(i = n-1-samplesBack+1; i < n-1; i++) {
				// initialise the poses and times to use
				geometry_msgs::Point p1 = poseBuffer[i].pose.position; //n-1 pose
				geometry_msgs::Point p2 = poseBuffer[n-1-samplesBack].pose.position;
				double t1 = poseBuffer[i].header.stamp.toSec();
				double t2 = poseBuffer[n-1-samplesBack].header.stamp.toSec();

				if((std::abs(p2.x + windowSpeed_x*(t1-t2)) > std::abs(p1.x+uncertainty_band_))
					&& (std::abs(p2.y + windowSpeed_y*(t1-t2)) > std::abs(p1.y+uncertainty_band_))
					&& (std::abs(p2.z + windowSpeed_z*(t1-t2)) > std::abs(p1.z+uncertainty_band_))) {
					// If this happens, we can state that we cannot use this frame. End loop
					allInBand = false;
					break;
				}
			}

			// this many steps back still worked. Awesome! Try one more if possible
			if(allInBand && n > samplesBack + 1) {
				samplesBack++;
				if(samplesBack > pose_memory_) {
					optimalSampleReached = true;
				}
			} else  {
				// Too bad, we have to use this many steps back. Exit loop!
				optimalSampleReached = true;
			}
		}

		return samplesBack;
	}

	void publishOdom(ros::Time time, geometry_msgs::Pose pose, geometry_msgs::Twist vel) {
		nav_msgs::Odometry odom_msg;

		odom_msg.header.stamp = time;

		odom_msg.pose.pose = pose;
		odom_msg.twist.twist = vel;

		odom_publisher_.publish(odom_msg);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "velocity_estimator");
	VelEstimator velocity_estimator;
	ros::spin();

	return 0;
}


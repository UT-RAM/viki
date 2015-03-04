/* The MIT License (MIT)

Copyright (c) 2015 Rutger Hendriks & Roald Looge

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

#include <vector>
#include "ros/ros.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"

#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"

#include "geometry_msgs/PoseStamped.h"
#include "calibrationParser.h"


//Publishers and broadcasters
ros::Publisher pub_info0, pub_info1;
tf::TransformBroadcaster* tf_br;

// Globals
CalibrationParser *parser;
sensor_msgs::CameraInfo info0, info1;
tf::Transform tr_cam0_imu;
unsigned images_received, pose_received;


void updateOverview()
{
	std::cout 	<< "\r" 
				<< "Images received: " << images_received << std::endl
				<< "Poses received: " << pose_received << std::flush;
}

void imageCallback(const sensor_msgs::Image image)
{
	// TODO Synchronize timestamps probably

	// TODO Queue images so that they can be provided at the same time

	// Publish camera info
	info0.header = image.header;
	info1.header = image.header;
	pub_info0.publish(info0);
	pub_info1.publish(info1);

	// Publish static tf transform regarding cam0 to imu rotation/translation
	tf_br->sendTransform(tf::StampedTransform(tr_cam0_imu, image.header.stamp, "imu4", "cam0"));

	// Give a nice overview
	images_received++;
	updateOverview();
}

void poseCallback(const geometry_msgs::PoseStamped pose)
{
	pose_received++;
	updateOverview();
}

void setupTF()
{
	std::vector<double> trans = parser->getTranslation();
	std::vector<double> rot = parser->getRotation();

	tf::Vector3 origin;
	tf::Matrix3x3 tf3d;

	origin.setValue(trans[0], trans[1], trans[2]);
	tf3d.setValue(rot[0], rot[1], rot[2], rot[3], rot[4], rot[5], rot[6], rot[7], rot[8]);

	// Construct Quaternion
	tf::Quaternion tfqt;
	tf3d.getRotation(tfqt);

	tr_cam0_imu.setOrigin(origin);
	tr_cam0_imu.setRotation(tfqt);

	ROS_INFO("Setup tf done");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "SLAM_sensor");

	ros::NodeHandle n;

	// Transform broadcaster setup
	tf::TransformBroadcaster _br;
	tf_br = &_br;

	// Register services and topics
	pub_info0 = n.advertise<sensor_msgs::CameraInfo>("/left/camera_info", 1000);
	pub_info1 = n.advertise<sensor_msgs::CameraInfo>("/right/camera_info", 1000);

	// Setup calibration data for publishing
	parser = new CalibrationParser();
	sensor_msgs::CameraInfo *cams[2] = {&info0, &info1};
	sensor_msgs::CameraInfo* c = parser->parseCalibrationCameraInfo(cams);
	info0 = *c;
	info1 = *(c+1);

	// setup TF
	setupTF();

	ROS_INFO("Started publisher node, waiting for raw images...");
	ros::spin();
	return 0;
}
/**
 * Node for estimating the position
 *
 * Uses a detector for detection of a marker and outputs the position of that marker
 * Input image + Detector (camera position) -> Output: Pose of detected marker, in world frame
 *
 * May 2015 - Robin Hoogervorst - RaM
 */
#include "ram_tracker_buddy/helper.cpp"
#include "detector/DetectorFactory.cpp"

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <dynamic_reconfigure/server.h>
#include <ram_tracker_buddy/RamTrackerBuddyCamPoseConfig.h>

#include <image_transport/image_transport.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/newPoseMeasurementamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ardrone_autonomy/Navdata.h>

#include <tf/transform_broadcaster.h>

#include <unistd.h>
#include <iostream>
#include <vector>
#include <list>
#include <boost/foreach.hpp>
#include <ctime>
#include <string>
#include <sstream>

using namespace std;
using namespace cv;

class PoseEstimator {
private:
    // Detectors
    AbstractDetector* detector;
    DetectorFactory*  dFactory;

    int seqId;
    int seqIdDebug;

    // Names used for topic information
    string window_name;
    string image_topic;
    string optitrack_name;


    geometry_msgs::Pose   pose;
    geometry_msgs::Pose   prevPos;
    geometry_msgs::Twist  speedEstimate;

    std::vector<geometry_msgs::PoseStamped*> poseBuffer;
    int max_pose_buffer_size;

    double position_covariance;
    double twist_covariance;
    double delay;

    float camera_yaw;
    float camera_pitch;
    float camera_roll;

    double camera_offset_x;

    ros::Time image_time;

    ros::NodeHandle n;
    ros::Subscriber image_sub;
    ros::Subscriber pos_sub;

    ros::Publisher  pose_pub;
    ros::Publisher  debug_direct_pose_pub;
    ros::Publisher  debug_pixel_pose_pub;

    // Dynamic reconfigure
    dynamic_reconfigure::Server<ram_tracker_buddy::RamTrackerBuddyCamPoseConfig> reconf_server;
    dynamic_reconfigure::Server<ram_tracker_buddy::RamTrackerBuddyCamPoseConfig>::CallbackType f;

    // tf frames for debugging in rviz
    tf::TransformBroadcaster br;
    tf::Transform JohnFrame;
    tf::Transform JohnCameraFrame;
    tf::Transform JohnMarkerFrame;

public:
    PoseEstimator() {
        ros::NodeHandle n("~");
        seqId = 0;

        // Start reading parameters
        window_name = "Camera pose estimator";
        if (!n.getParam("image", image_topic)) {
            ROS_WARN("No image topic specified");
            image_topic = "/image_raw";
        }

        if (!n.getParam("optitrack_name", optitrack_name)) {
            ROS_WARN("No optitrack name specified for base, defaulting to 'camera'");
            optitrack_name = "camera";
        }

        if (!n.getParam("camera_offset_x", camera_offset_x)) {
            camera_offset_x = .21;
        };

        // Get the calibration file setting
        std::string calibration_file;
        if (!n.getParam("camera_calibration", calibration_file)) {
          ROS_WARN("No valid camera calibration file specified, shutting down");
          ros::shutdown();
        }

        // Get a detector using the factory and the calibration filename
        string detector_name;
        DetectorFactory* dFactory = new DetectorFactory();
        if (n.getParam("detector", detector_name)) {
            detector = dFactory->get(detector_name, calibration_file);
            if (detector == NULL) {
                ROS_WARN("We could not generate a valid detector with name %s", detector_name.c_str());
                ros::shutdown();
            }
        }

        image_time = ros::Time::now();

        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 0;
        pose.orientation.w = 1;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;

        delay = 0;
        max_pose_buffer_size = 100000;

        camera_yaw    = 0 ;
        camera_pitch  = 0 ;
        camera_roll   = 0 ;

        // ROS subscribers / publishers
        image_sub     = n.subscribe(image_topic, 1, &PoseEstimator::imageCallback, this);
        pos_sub       = n.subscribe("/"+optitrack_name+"/unfiltered_pose", 1, &PoseEstimator::poseCallback, this);

        pose_pub      = n.advertise<geometry_msgs::PoseStamped>("/ram_tracker_buddy/marker_vision_odom", 1);

        // Dynamic reconfigure settings
        f = boost::bind(&PoseEstimator::reconfigure, this, _1, _2);
        reconf_server.setCallback(f);

        // OpenCV window for debugging
        cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    }

    ~PoseEstimator() {
        delete dFactory;

        //TODO: Clear the pose buffer;
    }

    /**
      * Helper function for publishing an TF frame which can be used in rviz,
      * Basically converts Eigen Objects to TF objects which can be used.
      */
    void publishTfFrame(tf::Transform t, Eigen::Vector3f p, Eigen::Matrix3f R, std::string parent, std::string child) {
        t.setOrigin(tf::Vector3(p.x(), p.y(), p.z()));
        t.setBasis(tf::Matrix3x3(R(0,0), R(0,1), R(0,2), R(1,0), R(1,1), R(1,2), R(2,0), R(2,1), R(2,2)));
        br.sendTransform(tf::StampedTransform(t, ros::Time::now(), parent.c_str(), child.c_str()));
    }

    /**
      * Helper function to create an H Matrix, which can be used to
      * easily do transformations.
      */
    Eigen::Matrix4f createHMatrix(Eigen::Matrix3f R, Eigen::Vector3f P) {
        Eigen::Matrix4f H;

        H.topLeftCorner(3,3) = R;
        H.topRightCorner(3,1) = P;
        H.bottomRows(1) << 0, 0, 0, 1;

        return H;
    }

    /**
      * Publish and calculate the world position of the detected marker,
      * Inputs are position and orientation measured from (and relative to) the camera
      * OptiTrack position is used to calculate the world coordinates
      */
    void publishPose(Eigen::Vector3f position, Eigen::Quaternionf orientation, geometry_msgs::Pose optiPose) {
        // Initialize messages
        geometry_msgs::Pose   pose_msg;

        /**
          * Start calculation right position compared to camera view
          * Consists of:
          *   - Small roll, pitch, yaw from camera compared to hull (can be calibrated using reconfigure)
          *   - 180 degree rotation around z-axis, to match the camera frame with world frame
          *   - Orientation of our camera view
          *
          * Rotations are defined in the Report properly! Look into there for further details
          */

        // Initialize objects for calculation
        Eigen::Vector3f P_B; // Position B from OptiTrack
        P_B << optiPose.position.x, optiPose.position.y, optiPose.position.z;
        Eigen::Vector3f P_CB_B((float)camera_offset_x, 0, 0); // Translation Camera in Drone frame
        Eigen::Matrix3f R_180z; // Fixed 180 degree rotation around z
        R_180z << -1,  0, 0,
                   0, -1, 0,
                   0,  0, 1;

        Eigen::Matrix3f R_0;
        R_0 << 1, 0, 0,  0, 1, 0,  0, 0, 1;
        Eigen::Matrix3f R_camaxis;
        R_camaxis << 0, 0, 1,   -1, 0, 0,   0, -1, 0;
        Eigen::Matrix3f R_markercam;
        R_markercam << 0, 0, 1,  0, 1, 0, -1, 0, 0;
        Eigen::Matrix3f R_MA_CB = R_markercam; // TODO: incorporate rotation of marker here!

        Eigen::Quaternionf qR_B(optiPose.orientation.w, optiPose.orientation.x, optiPose.orientation.y, optiPose.orientation.z);
        Eigen::Matrix3f R_CB_B = euler_to_rotMat(camera_yaw, camera_pitch, camera_roll) * R_camaxis; // Calbration rotation for the camera

        // Create homogeneous transformation
        Eigen::Matrix3f R_B = qR_B.toRotationMatrix() * R_180z;
        Eigen::Matrix4f H_B     = createHMatrix(R_B, P_B);
        Eigen::Matrix4f H_CB_B  = createHMatrix(R_CB_B, P_CB_B);
        Eigen::Matrix4f H_MA_CB = createHMatrix(R_MA_CB, position);
        Eigen::Matrix4f H_MA    = H_B * H_CB_B * H_MA_CB;

        // Set the right position
        pose_msg.position.x = H_MA(0,3);
        pose_msg.position.y = H_MA(1,3);
        pose_msg.position.z = H_MA(2,3);

        // Orientation:
        // Basically this should work, but it is very, very untested at the moment.
        Eigen::Matrix3f R_MA = H_MA.topLeftCorner(3,3);
        Eigen::Quaternionf new_or(R_MA);
        pose_msg.orientation.w = new_or.w();
        pose_msg.orientation.x = new_or.x();
        pose_msg.orientation.y = new_or.y();
        pose_msg.orientation.z = new_or.z();

        // Generate stamped message
        geometry_msgs::PoseStamped poseStamped;
        seqId++;
        poseStamped.header.seq = seqId;
        poseStamped.header.stamp = image_time;
        poseStamped.header.frame_id = "global";
        poseStamped.pose = pose_msg;

        // Finally, finally publish the message
        pose_pub.publish(poseStamped);

        // Tf transformations!! :D
        publishTfFrame(JohnFrame, P_B, R_B, "world", "john");
        publishTfFrame(JohnCameraFrame, P_CB_B, R_CB_B, "john", "john_camera");
        publishTfFrame(JohnMarkerFrame, position, R_MA_CB, "john_camera", "john_marker");
    }

    /**
      * Callback for dynamic reconfigure
      */
    void reconfigure(ram_tracker_buddy::RamTrackerBuddyCamPoseConfig &config, uint32_t level) {
        delay       = config.delay;

        float degToRad      = 3.14159 / 180;
        camera_yaw          = config.camera_yaw * degToRad;
        camera_pitch        = config.camera_pitch * degToRad;
        camera_roll         = config.camera_roll * degToRad;
    }

    /**
      * Callback when an image from the camera has been received
      * Uses the detector to find a position (marker dependent on the type of detector), then publishes the new found pose
      */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        image_time = ros::Time::now() - ros::Duration(delay);

        // Convert ROS image message to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr  = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image;
        image   = cv_ptr->image.clone();

        // Find the position of our drone, when the image was taken.
        // Delay can be calibrated using exposure_time. Maybe a look into further delay here can help
        geometry_msgs::PoseStamped* temppose = getPoseAtTime(image_time);
        // If we have no valid position, we can not make an accurate position estimate
        if (temppose == NULL) {
          return;
        }

        // Find and publish position
        if (detector->findPosition(image)) {
            publishPose(detector->getPosition(), detector->getOrientation(), temppose->pose);
        }

        imshow(window_name, image); // OpenCV call
        waitKey(1);
    }

    /**
      * Pose callback from the optitrack,
      * adds to the buffer, so we know the position at a given time moment (until the max buffer size) back
      */
    void poseCallback(const geometry_msgs::Pose msg) {
        //Check if we have reached the maximum buffer size
        if (poseBuffer.size() >= max_pose_buffer_size) {
            delete poseBuffer[0];
            poseBuffer.erase(poseBuffer.begin());
        }

        // Prepare a new stamped measurement object
        geometry_msgs::PoseStamped* newPoseMeasurement = new geometry_msgs::PoseStamped();
        newPoseMeasurement->pose = msg;
        newPoseMeasurement->header.seq       = seqId++;
        newPoseMeasurement->header.stamp     = ros::Time::now();
        newPoseMeasurement->header.frame_id  = "world";

        // Add our new object to the buffer
        poseBuffer.push_back(newPoseMeasurement);
    }

    /**
      * Returns the pose for a certain ROS time
      * Used to be able to get the position at the time the image was taken
      * Returns a pointer to PoseStamped within the buffer
      */
    geometry_msgs::PoseStamped* getPoseAtTime(ros::Time timeStamp) {
        if (poseBuffer.size() == 0) {
            ROS_WARN("PoseBuffer is emtpy, you cannot request a pose");
            return NULL;
        }

        // When asked timestamp in the future, return the latest known, but also throw a warning
        if (timeStamp > ros::Time::now()) {
            ROS_WARN("You asked for a position in the future, are you kidding me?");
            return poseBuffer[poseBuffer.size() - 1];
        }

        if (timeStamp > poseBuffer[poseBuffer.size() -1]->header.stamp) {
            return poseBuffer[poseBuffer.size() - 1];
        }

        // When asked for a timestamp before the buffer, return the first known but also throw a warning
        if (poseBuffer[0]->header.stamp > timeStamp) {
            ROS_WARN("First element in position buffer has a greater timestamp that has been asked. Returned the first element");
            return poseBuffer[0];
        }

        // We know we should have the position at the time asked now,
        // Let's search for it and return it, for the user to be a happy man
        std::vector<geometry_msgs::PoseStamped*>::iterator prevI;
        for (std::vector<geometry_msgs::PoseStamped*>::iterator i = poseBuffer.begin(); i != poseBuffer.end(); i++) {
            float timeDiff = (timeStamp - (*i)->header.stamp).toSec();
            if (timeDiff < 0) {
              return *prevI;
            }

            prevI = i;
        }

        // We should not be able to hit this code, so throw a warning if we do
        ROS_WARN("No Pose Stamped found for time %f", timeStamp.toSec());
        return NULL;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cam_pose_estimator");
    PoseEstimator pe;

    /**
      * Start the spinner. Use a multithreaded one,
      * to be able to update the pose while processing one image
      */
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}

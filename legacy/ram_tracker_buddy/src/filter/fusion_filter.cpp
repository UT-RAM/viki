/**
  * Fusion Filter class
  *
  * Most important class for this implementation. Houses a Kalman Filter for
  * filtering an external pose calculation together with the IMU information
  * of another.
  *
  * May 2015 - Robin Hoogervorst - RaM
  */

#include <ros/ros.h>
#include "ram_tracker_buddy/helper.cpp"
#include "predict_command.cpp"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>

#include <dynamic_reconfigure/server.h>
#include <ram_tracker_buddy/FusionFilterConfig.h>

#include <tf/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <boost/array.hpp>
#include <vector>
#include <stdlib.h>
#include <string>

class FusionFilter {

private:
    // State variables for Kalman Filter
    Eigen::Matrix<float, 6, 1>  state;           // (position{x,y,z}, speed{x,y,z})
    Eigen::Matrix<float, 6, 6>  covariance;

    // Covariances used for the Kalman filter
    double imu_cov;
    double pos_cov;
    double twist_cov;

    // Buffer used to hold the IMU information together with the state
    std::vector<PredictCommand*> predictBuffer;

    // Flags for if we use vision and/or IMU. Can be configured using dynamic reconfigure
    bool use_imu;
    bool use_vision;

    // Variables for IMU
    ros::Time current_time, last_time;
    Eigen::Vector3f imu_offset;
    float alpha_imu_bias;

    // Drone orientation in its own frame (populated from OptiTrack right now)
    Eigen::Quaternionf drone_orientation;

    Eigen::Vector3f marker_translation;
    geometry_msgs::PoseStamped last_vision_position;
    Eigen::Vector3f last_position;

    // Dynamic reconfigure stuff server and callback
    dynamic_reconfigure::Server<ram_tracker_buddy::FusionFilterConfig> reconf_server;
    dynamic_reconfigure::Server<ram_tracker_buddy::FusionFilterConfig>::CallbackType f;

    // Ros variables
    ros::NodeHandle n;
    ros::Timer pub_pose_timer;

    // Subscribers
    ros::Subscriber camera_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber drone_pose_sub;

    // Publishers
    ros::Publisher pose_pub;
    ros::Publisher imu_debug_pub;

    // Rviz visualisation
    tf::TransformBroadcaster br;
    tf::Transform JamesFrame;
    tf::Transform JamesImuFrame;
    tf::Transform JamesOptiTrackFrame;
    tf::Transform ImuGravityFrame;

public:
    FusionFilter() {
        //TODO: Read out the configuration values
        ROS_INFO("Initialising FusionFilter - RaM");

        //Initialize kalman matrices, this should converge quickly if the filter is working correctly
        state       <<  0, 0, 0, 0, 0, 0;
        covariance  <<  0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0;

        current_time  = ros::Time::now();
        last_time     = ros::Time::now();

        imu_cov = .1;
        pos_cov = .1;
        twist_cov = .1;
        imu_offset << 0, 0, 0;

        use_imu         = true;
        use_vision      = true;
        alpha_imu_bias  = .2;

        last_position << 0, 0, 0;

        double marker_offset_y;
        if (!n.getParam("marker_offset_y", marker_offset_y)) {
            marker_offset_y = -.24;
        }

        marker_translation << 0, (float) marker_offset_y, 0;

        // Speed position from the camera
        camera_sub        = n.subscribe("/ram_tracker_buddy/marker_vision_odom", 1, &FusionFilter::camodom_callback, this);

        // IMU from the drone
        imu_sub           = n.subscribe("/James/ardrone/imu", 1, &FusionFilter::imu_callback, this);

        // Optitrack position. Is used for orientation. Ideally this should not be necessary
        drone_pose_sub    = n.subscribe("/James/unfiltered_pose", 1, &FusionFilter::pose_callback, this);

        // Timer event for publishing pose, publishes as 100 Hz
        pub_pose_timer = n.createTimer(ros::Duration(.01), &FusionFilter::publishPose, this);

        // Bind the reconfigure function to the reconfigure server for ROS
        f = boost::bind(&FusionFilter::reconfigure, this, _1, _2);
        reconf_server.setCallback(f);

        // Publishers for filtered position and debug options
        pose_pub      = n.advertise<geometry_msgs::Pose>("/James/tracker_buddy_pose", 1);
        imu_debug_pub = n.advertise<sensor_msgs::Imu>("/ram_tracker_buddy/debug/compensated_imu", 1);
    }

    /**
      * Callback from the OptiTrack, populates the orientation for use of calculations,
      * publishes also an tf frame for debug purposes
      */
    void pose_callback(const geometry_msgs::Pose msg) {
        Eigen::Quaternionf optitrack_orientation(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
        drone_orientation = optitrack_orientation;

        Eigen::Vector3f optitrack_position ( msg.position.x, msg.position.y, msg.position.z);
        publishTfFrame(JamesOptiTrackFrame, optitrack_position, optitrack_orientation.toRotationMatrix(), "world", "james_optitrack");
    }

    /**
      * Helper function for easy generation of an transformation matrix
      */
    Eigen::Matrix4f createHMatrix(Eigen::Matrix3f R, Eigen::Vector3f P) {
      Eigen::Matrix4f H;

      H.topLeftCorner(3,3)  = R;
      H.topRightCorner(3,1) = P;
      H.bottomRows(1)       << 0, 0, 0, 1;

      return H;
    }

    /**
      * Helper function for publishing a TFframe
      * These can be visualised using Rviz
      */
    void publishTfFrame(tf::Transform t, Eigen::Vector3f p, Eigen::Matrix3f R, std::string parent, std::string child) {
        t.setOrigin(tf::Vector3(p.x(), p.y(), p.z()));
        t.setBasis(tf::Matrix3x3(R(0,0), R(0,1), R(0,2), R(1,0), R(1,1), R(1,2), R(2,0), R(2,1), R(2,2)));
        br.sendTransform(tf::StampedTransform(t, ros::Time::now(), parent.c_str(), child.c_str()));
    }

    /**
     * Callback for receiving a camera image, runs at a slower publish rate than the IMU
     * Compares to the update step of the Kalman filter\
     *
     * Rotations are defined properly in the Report! (Collaborative pose esitmation of
     * an UAV using vision and IMU data, by Robin Hoogervorst). Check that
     * for more information if the rotations are unclear.
     */
    void camodom_callback(const geometry_msgs::PoseStamped msg) {
        if (!use_vision) return;

        Eigen::Matrix3f R_180z; // Fixed 180 degree rotation around z
        R_180z << -1,  0, 0,
                  0, -1, 0,
                  0,  0, 1;

        Eigen::Matrix3f R_m;
        R_m << 1, 0, 0,  0, 0, 1,  0, -1, 0;

        Eigen::Matrix3f R_0;
        R_0 << 1, 0, 0,  0, 1, 0,  0, 0, 1;

        // Generate Eigen objects from the data that we have..
        Eigen::Quaternionf qR_ma(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
        Eigen::Vector3f P_MA (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

        /** This gets the orientation from the marker to the drone. For now, this does
          * not use the orientation from the state or the measurment, but from
          * the OptiTrack. TODO: Incorporate the orientation into the state,
          * and use that orientation to do this calculation..
          */
        Eigen::Matrix4f H_MA = createHMatrix(drone_orientation.toRotationMatrix() * R_180z * R_m, P_MA);

        Eigen::Matrix3f R_MA_A  = R_m;
        Eigen::Vector3f P_MA_A  = marker_translation;
        Eigen::Matrix4f H_A_MA  = createHMatrix(R_MA_A.inverse(), - R_MA_A.inverse() * P_MA_A);

        Eigen::Matrix4f H_A = H_MA * H_A_MA;
        Eigen::Vector3f current_position(H_A(0,3), H_A(1,3), H_A(2,3));

        if (last_vision_position.header.stamp > msg.header.stamp) {
            ROS_WARN("New image has later timeStamp than image received. Please check
                     the order of receiving of the images..");
        }
        double dt = (msg.header.stamp - last_vision_position.header.stamp).toSec();
        Eigen::Vector3f v = (current_position - last_position) / (dt);
        last_position << current_position.x(), current_position.y(), current_position.z();
        last_vision_position = msg;

        // Convert to eigen matrix
        Eigen::Matrix<float, 6, 1> position_measurement;
        position_measurement << current_position.x(), current_position.y(), current_position.z(),
                                v.x(), v.y(), v.z();

        // When we only use the camera, we can not use the update steps from the equation,
        // Therefore, we will just force our position measurement
        if (!use_imu && use_vision) {
          state << position_measurement;
          return;
        }

        adaptIMUbias(v, drone_orientation.toRotationMatrix() * R_180z, dt);

        // Start synchronisation! First backtrack the state to the last IMU
        // step before the image timestamp
        bool synchronise = true;
        if (predictBuffer.size() > 0 && use_imu && synchronise) {
          while (predictBuffer.size() > 0 && predictBuffer[0]->timeStamp < msg.header.stamp) {
              delete predictBuffer[0];
              predictBuffer.erase(predictBuffer.begin());
          }

          state       = predictBuffer[0]->preState;
          covariance  = predictBuffer[0]->preCovariance;
        }

        if (!synchronise) {
          while (predictBuffer.size() > 0 && predictBuffer[0]->timeStamp < msg.header.stamp) {
              delete predictBuffer[0];
              predictBuffer.erase(predictBuffer.begin());
          }
        }

        if (predictBuffer.size() == 0) {
            ROS_WARN("Prediction buffer has been cleared totally. This is probably wrong,
                     please check if timestamps are correct!");
        }

        // Construct the measurment covariance matrix
        Eigen::Matrix<float, 6, 6> measurement_covariance;
        float tcx = twist_cov;
        float tcy = twist_cov;
        float tcz = twist_cov;

        measurement_covariance << pos_cov, 0, 0, 0, 0, 0,
                                  0, pos_cov, 0, 0, 0, 0,
                                  0, 0, pos_cov, 0, 0, 0,
                                   0,  0,  0, tcx, 0, 0,
                                   0,  0,  0, 0, tcy, 0,
                                   0,  0,  0, 0, 0, tcz;

        // Use this backtracked state to update the position using the measurement
        updatePosition(position_measurement, measurement_covariance);

        // Re-apply the buffered IMU data from the buffer
        if (synchronise) {
          std::vector<PredictCommand*>::iterator it = predictBuffer.begin();
          for (it; it != predictBuffer.end(); it++) {
              predictState(*it);
          }
        }
    }

    // Adapt the IMU bias if our speed is zero
    void adaptIMUbias(Eigen::Vector3f v, Eigen::Matrix3f imu_rotation, float dt) {
        if (dt > 1) return; // When the dt is too large, we will not use this measurement

        Eigen::Vector3f state_v(state(3), state(4), state(5));
        Eigen::Vector3f rotated_state_v = imu_rotation.inverse() * state_v;
        Eigen::Vector3f diff_v = imu_rotation.inverse() * (state_v - v);

        float v_threshold = 0;
        for (int i=0; i<3; i++) {
             if (abs(diff_v[i]) > v_threshold && dt < 1) {
                imu_offset[i] += (diff_v[i] * dt) * alpha_imu_bias;
             }
        }

        if (alpha_imu_bias  == 0) {
          ROS_WARN("alpha of IMU bias has been set to 0. No offset has been applied!");
        }
    }

    /**
      * Update step of kalman filter,
      * Uses the measurment from the camera, with a generated speed from the camera
      */
    void updatePosition(Eigen::Matrix<float, 6, 1> measurement, Eigen::Matrix<float, 6, 6> measurement_covariance) {
        // Measurment matrix from Kalman Filter, H
        Eigen::Matrix<float, 6, 6> H;
        H <<  1, 0, 0, 0, 0, 0,
              0, 1, 0, 0, 0, 0,
              0, 0, 1, 0, 0, 0,
              0, 0, 0, 1, 0, 0,
              0, 0, 0, 0, 1, 0,
              0, 0, 0, 0, 0, 1;

        // Difference calculation
        Eigen::Matrix<float, 6, 1> y = measurement - H * state;

        Eigen::Matrix<float, 6, 6> S;
        Eigen::Matrix<float, 6, 6> K;
        Eigen::Matrix<float, 6, 6> I;
        I <<  1,  0,  0,  0,  0,  0,
              0,  1,  0,  0,  0,  0,
              0,  0,  1,  0,  0,  0,
              0,  0,  0,  1,  0,  0,
              0,  0,  0,  0,  1,  0,
              0,  0,  0,  0,  0,  1;

        // Calculate Kalman gain
        S     = H * covariance * H.transpose() + measurement_covariance;
        K     = covariance * H.transpose() * S.inverse();

        // Actual update
        state         = state + K * y;
        covariance    = (I - K * H) * covariance;
    }

    /**
      * Callback for receiving IMU data,
      * Compares to prediction step of the Kalman filter
      */
    void imu_callback(const sensor_msgs::Imu msg)
    {
        // Calculate dt, based on ROS time
        current_time  = ros::Time::now();
        double dt     = (current_time - last_time).toSec();

        // Generate a Eigen::Vector from the message
        Eigen::Vector3f lin_acc(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);

        if (!use_imu) return;

        // Construct covariance matrix
        Eigen::Matrix3f lin_cov;
        lin_cov <<   imu_cov,   0,   0,
                      0,  imu_cov,   0,
                      0,   0,  imu_cov;


        // Rotation of IMU compared to the drone. Frames of reference are the same for now,
        // But using this code, this can be easily adapted if the IMU has another frame of reference
        Eigen::Matrix3f R_IA_A;
        R_IA_A << 1, 0, 0,   0, 1, 0,   0, 0, 1;
        Eigen::Vector3f P_IA_A(0, 0, 0);
        Eigen::Matrix4f H_IA_A = createHMatrix(R_IA_A, P_IA_A);

        // Rotation of the drone: 180degree rotation compared to OptiTrack + orientation from that
        Eigen::Matrix3f R_180z;
        R_180z << -1, 0, 0,  0, -1, 0,  0, 0, 1;
        Eigen::Matrix3f R_A = drone_orientation.toRotationMatrix() * R_180z;

        // IMU rotation in world frame
        Eigen::Matrix3f R_IA = R_A * R_IA_A;

        // Compute the actual acceleration from the measurement (subtract the offset and the gravity)
        Eigen::Vector3f g(0, 0, 9.8);
        Eigen::Vector3f world_acc = R_IA * (lin_acc - imu_offset) - g;

        // Generate a new predict command and store it in the buffer, for use in backtracking
        PredictCommand* pc = new PredictCommand(state, covariance, world_acc, lin_cov, dt, current_time);
        predictBuffer.push_back(pc);

        // Apply the prediction immediately as well
        predictState(pc);

        // Store the current time for calculation of dt the next time
        last_time = current_time;

        // Debugging for IMU
        publishTfFrame(JamesImuFrame, P_IA_A, R_IA, "world", "james_imu");

        sensor_msgs::Imu imu_msg;
        imu_msg.header.seq = 1;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "world";

        imu_msg.linear_acceleration.x = world_acc[0];
        imu_msg.linear_acceleration.y = world_acc[1];
        imu_msg.linear_acceleration.z = world_acc[2];

        imu_debug_pub.publish(imu_msg);
    }

    /**
     *  Predict a state step, using the Kalman filter equations
     *  Linear acceleration is in body fixed frame of the drone and gets transformed here
     *  using the orientation of the drone known to the filter
     */
    void predictState(PredictCommand* pc)
    {
        Eigen::Matrix<float, 6, 6> F;   // State matrix for kalman
        F <<  1,  0,  0, pc->dt,  0,  0,
              0,  1,  0,  0, pc->dt,  0,
              0,  0,  1,  0,  0, pc->dt,
              0,  0,  0,  1,  0,  0,
              0,  0,  0,  0,  1,  0,
              0,  0,  0,  0,  0,  1;

        Eigen::Matrix<float, 6, 3> B;   // Input conversion matrix for Kalman
        B <<  0,  0,  0,
              0,  0,  0,
              0,  0,  0,
              pc->dt,  0,  0,
              0,  pc->dt,  0,
              0,  0, pc->dt;

        Eigen::Matrix<float, 6, 6> Q;
        Q <<  0,  0,  0,          0,          0,          0,
              0,  0,  0,          0,          0,          0,
              0,  0,  0,          0,          0,          0,
              0,  0,  0,  pc->lin_cov(0),          0,          0,
              0,  0,  0,          0,  pc->lin_cov(4),          0,
              0,  0,  0,          0,          0,  pc->lin_cov(8);

        // Kalman prediction equations
        state       = F * state + B  * pc->world_lin_acc;
        covariance  = F * covariance * F.transpose() + Q;
    }


    /**
      * Publishes the pose of the current state, in the world coordinate system
      * This is independent of the publish rate of the subscribed topics and
      * publishes based on a ROS timer.
      */
    void publishPose(const ros::TimerEvent& event) {
        geometry_msgs::Pose msg;
        msg.position.x    = state(0);
        msg.position.y    = state(1);
        msg.position.z    = state(2);

        msg.orientation.w = drone_orientation.w();
        msg.orientation.x = drone_orientation.x();
        msg.orientation.y = drone_orientation.y();
        msg.orientation.z = drone_orientation.z();

        // We do not want to publish stuff if we don't have inputs
        if (use_imu || use_vision) {
          pose_pub.publish(msg);
        }

        // Tf Frame publishing for RVIZ functionality
        Eigen::Vector3f P(msg.position.x, msg.position.y, msg.position.z);
        publishTfFrame(JamesFrame, P, drone_orientation.toRotationMatrix(), "world", "james");
    }

    /**
      * Dynamic reconfigure callback
      */
    void reconfigure(ram_tracker_buddy::FusionFilterConfig &config, uint32_t level) {
        use_imu         = config.use_imu;
        use_vision      = config.use_vision;

        if (use_imu) { // reset the times for the imu calculation
            last_time     = ros::Time::now();
            current_time  = ros::Time::now();
        }

        imu_cov         = config.imu_covariance;
        pos_cov         = config.position_covariance;
        twist_cov       = config.twist_covariance;

        alpha_imu_bias  = config.alpha_imu_bias;
    }

};


// Entry point, initialises ROS for this node and creates the FusionFilter
int main(int argc, char** argv) {
    ros::init(argc, argv, "fusion_filter");
    FusionFilter ff;

    ros::spin();
    return 0;
}

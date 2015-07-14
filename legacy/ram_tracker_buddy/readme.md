# Ram tracker buddy package

The ram tracker buddy package is made by Robin Hoogervorst for his Bachelor Assignment, which has been done at the Robotics and Mechatronics (RaM) group at the University of Twente. This report belonging to this is "Collaborative pose estimation of an UAV using vision and IMU data". This report will explain in greater detail the goal and purpose of this implementation.

## Nodes

This package contains two nodes:

- Camera pose estimator (cam_pose_estimator)
- Fusion Filter (fusion_filter)

### Camera pose estimator
This takes an image as input and will provide the 3d pose of an marker in this image. Which marker depends on the Detector that is used. A detector can be made by extending the AbstractDetector and adding it to the DetectorFactory. This way, own implementations of the Detector can be easily made.

### Fusion filter
This filters a pose of the marker on the UAV together with the IMU information of the UAV. This will provide a final 3d pose.

## Running
Prerequisites:
Make sure an node is running for determining the exact pose. In this implementation, the OptiTrack is used for this, together with the [mocap_optitrack](https://github.com/ros-drivers/mocap_optitrack/tree/master/src) package. There should be two trackables available:

- First trackable is for the camera position, of which the name can be configured in the launch file. The driver configuration should push the pose of this object into the '/_name_/unfiltered_pose' topic.

- The main UAV is in this implementation called _James_. Therefore, its pose should be published at '/James/unfiltered_pose'.



The easiest way to run the code is using a launch file.

```
<launch>

  <node name="camera_stream" pkg="ram_tracker_buddy" type="cam_pose_estimation" output="screen">
    <param name="detector"        value="APRIL" />
    <param name="optitrack_name"  value="John" />
    <param name="image"           value="/John/ardrone/image_raw" />
    <param name="camera_calibration" value="<<XML file with calibration>>" />
    <param name="camera_offset_x" value=".21" />
  </node>

  <node name="filtered" pkg="ram_tracker_buddy" type="fusion_filter" output="screen">
    <param name="use_imu"       value="True" />
    <param name="use_vision"    value="True" />
    <param name="marker_offset_y" value="-.24" />
  </node>

</launch>
```

A detailed explanation of all the parameters is shown below. The nodes in the launch file can be run independently from each other as well. The launch file above can be runned using:
```
  roslaunch ram_tracker_buddy drone_vision.launch
```
Of course, you can run your own launch file, which also launches these nodes. Please keep in mind then that the type should be _cam_pose_estimation_ for launch of the camera pose estimator and it should be _fusion_filter_ if the fusion filter should be launched.

The filtered pose will be published on '/James/tracker_buddy_pose'. The pose of the marker will be puslibhed at '/ram_tracker_buddy/marker_vision_odom'. (Please keep in mind that this isn't an odom, but a PoseStamped).

Control of the UAVs can be done in basically any way you like. You may use the topics above to use the result of this implementation for use in the controller of your choice.

## Configuration

### Launch files
The following parameters are available for the cam_pose_estimation:
- **detector**: This is the slug of the detector that will be used, at the moment (june 2015) only "APRIL" and "DOTS" are supported. For further reference, look at the report. This can be extended by creating an extra detector and adding this to the factory.

- **optitrack_name**: This is the name of the object for the OptiTrack pose input. As pose input for the camera position, /'name'/unfiltered_pose will be used.
- **image**. The image stream (ROS topic) that is to be used
- **camera_calibration**: The exact location to an XML file that houses the camera calibration. This should be in OpenCV format. Examples are shown below
- **camera_offset_x**. The offset of the camera compared to the OptiTrack position on the x-axis in meters. For the parrot AR.Drone this is 21 cm. Camera offset on other axis is _not_ possible and has not been implemented!

For the ram_tracker_buddy, the following parameters are available in the launch file:

- **use_imu**: Flag if the IMU is to be used
- **use_vision**: Flag if vision measurements should be used
- **marker_offset_y**: Offset of the marker in the y direction of the drone.

### Dynamic reconfigure
Furthermore, there are some parameters that can be configured using dynamic reconfigure. These parameters can be changed dynamically during runtime. To reach these settings, execute the following command while the nodes are running.
```
rosrun rqt_reconfigure rqt_reconfigure
```
or run
```
rqt
```
and open the dynamic reconfigure window.

For the camera pose estimator, some dynamic settings were experimented with. In the end, these were all put to 0 for the final results. However, they are still incorporated since they might be useful in the future. The following parameters are then available for the camera pose estimator:

- **delay**: Estimated delay of the receiving of the images. Does **NOT** include the processing time. That will be compensated automatically. In practice, it works best if this is set to be 0.
- **camera_yaw, camera_roll, camera_pitch**. Relative rotation of the camera compared to the UAV. This can be used to calibrate a camera that is not exactly centered. In practice, these were put to 0 as well.

For the fusion filter, there are some really important parameters:

- **use_imu**: to dynamically switch on and off the use of IMU information for use in the estimate.
- **use_vision**: to dynamically switch on and off the use of vision measurements for use in the estimate. (The vision estimate will still be running, they will just be neglected by the filter)

_Note: When both use_imu and use_vision are turned off, no data will be published_

- **position_covariance**: The covariance of the position to be used
- **twist_covariance**: The covaiance to be used for the estimated velocity using the vision measurements.
- **imu_covariance**: The covariance to use for the IMU.
_Note: Using these settings, it is easy to calibrate the filter._

- **alpha_imu_bias**: The alpha factor to be used when changing the bias of the IMU. A low value says that the bias will almost not be adapted, while a high value will change the bias to the exact value estimated by the vision. Usually, it is useful to start a high value, to change the bias quickly to a setpoint. After that, it can be useful to switch to a low value to be able to rely on the IMU more.

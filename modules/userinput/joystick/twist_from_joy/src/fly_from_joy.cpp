/*! \file fly_from_joystick.cpp 
* \brief Fly From joystick executable for the RAM package
*
* This package enables you to control a Parrot AR.Drone from the keyboard of your computer or laptop. It publishes to a number of topics:
* - /ardrone/takeoff
* - /ardrone/land
* - /ardrone/reset
* - /cmd_vel
*
* This package is inspired by another joystick teleop package from https://github.com/tu-darmstadt-ros-pkg.
*/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h> 

/*! Teleoperation class, enabling the use of a joystick to for remote control */
class Teleop
{
private:
  ros::NodeHandle node_handle_;
  ros::Subscriber joy_subscriber_;
  ros::Publisher velocity_publisher_;
  ros::Publisher land_publisher_;
  ros::Publisher takeoff_publisher_;
  ros::Publisher reset_publisher_;
  geometry_msgs::Twist velocity_;
  std_msgs::Empty empty_;
  std::string prefix; /**< This seems unused */

  struct Axis
  {
    int axis;
    double max;
  };

  struct Button
  {
    int button;
  };

  struct {
    Axis x;
    Axis y;
    Axis z;
    Axis yaw;
  } axes_;

  struct
  {
    Button slow;
    Button land;
    Button takeoff;
    Button reset;
    Button toggleControl;
  } buttons_;

  double slow_factor_;
  bool enable_control_;

public:
  Teleop()
  {
    ros::NodeHandle params("~");

    // Default values
    axes_.x.axis = 0;
    axes_.x.max = 2.0;
    axes_.y.axis = 0;
    axes_.y.max = 2.0;
    axes_.z.axis = 0;
    axes_.z.max = 2.0;
    axes_.yaw.axis = 0;
    axes_.yaw.max = 90.0*M_PI/180.0;
    buttons_.slow.button = 0;
    buttons_.land.button = 0;
    buttons_.reset.button = 0;
    buttons_.takeoff.button = 0;
    slow_factor_ = 0.2;
    enable_control_ = true;
    prefix = "";

    // Read parameters from file
    params.getParam("x_axis", axes_.x.axis);
    params.getParam("y_axis", axes_.y.axis);
    params.getParam("z_axis", axes_.z.axis);
    params.getParam("yaw_axis", axes_.yaw.axis);
    params.getParam("x_velocity_max", axes_.x.max);
    params.getParam("y_velocity_max", axes_.y.max);
    params.getParam("z_velocity_max", axes_.z.max);
    params.getParam("yaw_velocity_max", axes_.yaw.max);
    params.getParam("slow_button", buttons_.slow.button);
    params.getParam("reset_button", buttons_.reset.button);
    params.getParam("land_button", buttons_.land.button);
    params.getParam("takeoff_button", buttons_.takeoff.button);
    params.getParam("toggle_control_button", buttons_.toggleControl.button);
    params.getParam("slow_factor", slow_factor_);
    params.getParam("enable_stick_control_init", enable_control_);
    params.getParam("prefix", prefix);

    // Init subscribers
    joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 1, boost::bind(&Teleop::joyCallback, this, _1));
    velocity_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("cmd_vel_joy", 1);
    land_publisher_ = node_handle_.advertise<std_msgs::Empty>("ardrone/land", 1);
    takeoff_publisher_ = node_handle_.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
    reset_publisher_ = node_handle_.advertise<std_msgs::Empty>("ardrone/reset",1);
  }

  ~Teleop()
  {
    stop();
  }

  //! Callback for joy messages. Get the axes value and check if interesting buttons are pressed.
  void joyCallback(const sensor_msgs::JoyConstPtr& joy)
  {
    velocity_.linear.x  = getAxis(joy, axes_.x.axis)   * axes_.x.max;
    velocity_.linear.y  = getAxis(joy, axes_.y.axis)   * axes_.y.max;
    velocity_.linear.z  = getAxis(joy, axes_.z.axis)   * axes_.z.max;
    velocity_.angular.z = getAxis(joy, axes_.yaw.axis) * axes_.yaw.max;
    if (getButton(joy, buttons_.slow.button)) {
      velocity_.linear.x  *= slow_factor_;
      velocity_.linear.y  *= slow_factor_;
      velocity_.linear.z  *= slow_factor_;
      velocity_.angular.z *= slow_factor_;
    }
    if (getButton(joy, buttons_.land.button)) {
      land_publisher_.publish(empty_);
    }
    if (getButton(joy, buttons_.takeoff.button)) {
      takeoff_publisher_.publish(empty_);
    }
    if (getButton(joy, buttons_.reset.button)) {
      reset_publisher_.publish(empty_);
    }
    if (getButton(joy, buttons_.toggleControl.button)) {
    	if(enable_control_)
    	{
    		enable_control_ = false;
    	}
    	else 
    	{
    		enable_control_ = true;
    	}
    }

    // Only publish control if control is enabled.
    if(enable_control_)
    {
    	velocity_publisher_.publish(velocity_);
    }
  }

  //! Helper function to get the right axis and enable inversion.
  sensor_msgs::Joy::_axes_type::value_type getAxis(const sensor_msgs::JoyConstPtr& joy, int axis)
  {
    if (axis == 0) return 0;
    sensor_msgs::Joy::_axes_type::value_type sign = 1.0;
    if (axis < 0) { sign = -1.0; axis = -axis; }
    if ((size_t)axis > joy->axes.size()) return 0;
    return sign * joy->axes[axis - 1];
  }

  //! Helper function to get the right button
  sensor_msgs::Joy::_buttons_type::value_type getButton(const sensor_msgs::JoyConstPtr& joy, int button)
  {
    if (button <= 0) return 0;
    return joy->buttons[button - 1];
  }

  //! Publish hover on stop.
  void stop()
  {
    velocity_ = geometry_msgs::Twist();
    velocity_publisher_.publish(velocity_);
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fly_from_joystick");
  
  Teleop teleop;
  ros::spin();

  return 0;
}


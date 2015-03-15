/*! \file controller.cpp
* \brief controller executable for the RAM package
*
* This executable implements a PD controller. Gains, non-linear actions, velocity-damping and integrative actions can be set through a Python UI.
*
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Empty.h>
#include <math.h> 
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <ram/nonlinearity.h>
#include <ram/gains.h>
#include <cmath>
#include <iostream>

//! Controller class.
class Control
{
private:
  ros::NodeHandle node_handle_;
  ros::Subscriber sim_subscriber_; // Very untested at the moment. Old.
  ros::Subscriber setpoint_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Publisher velocity_publisher_;

  /* Non linear control part */
  ros::Subscriber nonlin_subscriber_;
  float nonlinx;
  float nonliny;
  float nonlinz;
  float xoff;
  float yoff;
  /* End non linear control part */

  /* ADDITIONAL PUBLISHER FOR GRAPHING PURPOSES */
  ros::Publisher p_publisher_;
  ros::Publisher d_publisher_;
  ros::Publisher x_publisher_;
  ros::Publisher yaw_publisher_;
  ros::Publisher errorx_publisher_; 
  ros::Publisher errordx_publisher_;
  ros::Publisher foaw_publisher_;
  ros::Publisher velocity_damping_publisher_;
  /* END ADDITIONAL PUBLISHERS */

  /* Memory for error calculations*/
  std::vector <double> errorsx_; // Previous errors in x
  std::vector <double> errorsy_; // Previous errors in y
  std::vector <double> errorsz_; // Previous errors in z
  std::vector <double> errorsyaw_; // Previous errors in z
  std::vector <double> times_; // Corresponding times
  std::vector <geometry_msgs::Twist>  speeds_; // speeds
  std::vector <double> yaws_;

  /* Filter parameters */
  // FOAW
  double uncertainty_band_; // Max band around pose to find line fitting
  int error_memory_; // Max errors stored

  // LOW PASS
  double T_, dt_;

  // STATE ESTIMATION
  double K_;

  // Messages
  geometry_msgs::Twist velocity_;
  std_msgs::Empty empty_;

  // Control parameters
  bool hovermode_;
  bool velocity_damping_;
  bool i_action_;
  double hover_treshold_;

  ros::Subscriber gain_subscriber_;
  struct 
  {
    double p_z;
    double d_z;
    double p_translational;
    double d_translational;
    double p_rotational;
    double d_rotational;
    double velocity;
    double v_limit;    
    double i;
  } Gains;

  struct
  {
    double x;
    double y;
    double z;
    double yaw;
  } Setpoint;

  struct 
  {
    double x_translational;
    double y_translational;
    double z_translational;
    double z_rotational;
  } Error;

  struct 
  {
    double x_translational;
    double y_translational;
    double z_translational;
    double z_rotational;
  } ErrorDot;

  // Integral action
  struct
  {
    double x;
    double y;
  } isum;

  // general variables
  bool simulation;
  double previous_publish_time_;
  int freq_;
  double yaw_; // Screw quaternions

public:
  Control()
  {
    ROS_INFO("Initializing controller");
    ros::NodeHandle params("~");

    // If simulation is true, get data feedback from simulation and run simulationCallback. Otherwise, use OptiTrack
    // This feature is untested. It is very untested at the moment.
    simulation = false;
    params.getParam("simulation",simulation);
    if(simulation) {
      sim_subscriber_ = node_handle_.subscribe("/gazebo/model_states", 1, &Control::simulationCallback, this); 
      ROS_INFO("SIMULATION MODE"); 
    }
    else { 
      odom_subscriber_ = node_handle_.subscribe("filtered_state", 1, &Control::odomCallback, this);  
      ROS_INFO("CONTROL MODE");
    }

    // Velocity publisher for velocity damping.
    velocity_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("cmd_vel_controller", 1);

    /* Non linear part */
    nonlin_subscriber_ = node_handle_.subscribe("nonlinearity", 1, &Control::nonlinCallback, this);
    nonlinx = 0;
    nonliny = 0;
    nonlinz = 0;
    xoff = 0;
    yoff = 0;

    /* ADDITIONAL PUBLISHER FOR GRAPHING PURPOSES */
    d_publisher_ = node_handle_.advertise<std_msgs::Float32>("control_d",1);
    p_publisher_ = node_handle_.advertise<std_msgs::Float32>("control_p",1);
    x_publisher_ = node_handle_.advertise<std_msgs::Float32>("tot_x",1);
    errorx_publisher_ = node_handle_.advertise<std_msgs::Float32>("error_x",1);
    errordx_publisher_ = node_handle_.advertise<std_msgs::Float32>("errord_x",1);
    yaw_publisher_ = node_handle_.advertise<std_msgs::Float32>("yaw",1);
    foaw_publisher_ = node_handle_.advertise<std_msgs::Int32>("foaw_d",1);
    velocity_damping_publisher_ = node_handle_.advertise<std_msgs::Float32>("velocity_damping",1);
    /* END ADDITIONAL PUBLISHERS */

    // Subscriber for setpoint changes
    setpoint_subscriber_ = node_handle_.subscribe("setpoint", 1, &Control::setpointCallback, this);

    // Initial values
    previous_publish_time_ = 0;
    freq_ = 50;
    params.getParam("publish_rate", freq_);
    
    // Hover mode
    hovermode_ = false;
    hover_treshold_ = 0.1;
    params.getParam("hovermode",hovermode_);
    params.getParam("hover_treshold",hover_treshold_);
    
    // Velocity damping
    velocity_damping_ = false;
    params.getParam("velocity_damping", velocity_damping_);

    /* Filtering */
    // FOAW
    error_memory_ = 15;
    uncertainty_band_ = 0.01;
    params.getParam("error_memory", error_memory_);
    params.getParam("uncertainty_band", uncertainty_band_);

    // STATE ESTIMATION
    K_ = 70;
    params.getParam("K", K_);

    // LOWPASS
    T_ = 0.1; //sec
    dt_ = 0.03; //ms

    
    // Initial values for setpoint
    params.getParam("setpoint_x", Setpoint.x);
    params.getParam("setpoint_y", Setpoint.y);
    params.getParam("setpoint_z", Setpoint.z);
    params.getParam("setpoint_yaw", Setpoint.yaw);
    // Controller gains
    params.getParam("gain_p_translational", Gains.p_translational);
    params.getParam("gain_d_translational", Gains.d_translational);
    params.getParam("gain_p_z", Gains.p_z);
    params.getParam("gain_d_z", Gains.d_z);
    params.getParam("gain_p_rotational", Gains.p_rotational);
    params.getParam("gain_d_rotational", Gains.d_rotational);
    params.getParam("gain_velocity", Gains.velocity);
    params.getParam("limit_velocity",Gains.v_limit);    

    // I action
    i_action_ = false;
    params.getParam("gain_i",Gains.i);
    params.getParam("i_action",i_action_);
    isum.x = 0;
    isum.y = 0;
    ROS_INFO("%f",isum.x);

    gain_subscriber_ = node_handle_.subscribe("gains", 1, &Control::gainCallback, this);
  }

  ~Control()
  {
    // Publish empty msg as last message when closed
    velocity_ = geometry_msgs::Twist();
    velocity_publisher_.publish(velocity_);
  }

  //! Gain callback. Gains are changed when this function is called.
  void gainCallback(const ram::gains msg)
  {
    Gains.p_translational = msg.p_trans;
    Gains.d_translational = msg.d_trans;
    Gains.p_rotational = msg.p_rot;
    Gains.d_rotational = msg.d_rot;
    Gains.p_z = msg.p_z;
    Gains.d_z = msg.d_z;
    Gains.i = msg.i_action;
    Gains.velocity = msg.v_damping;
    Gains.v_limit = msg.v_limit;    
    i_action_ = msg.i_enabled;
    velocity_damping_ = msg.v_enabled;
    ROS_INFO("GAINS CHANGED");
  }

  //! Nonlin Callback. The nonlinear part is disabled or enabled when this function is called
  void nonlinCallback(const ram::nonlinearity msg)
  {
      nonlinx = msg.x;
      nonliny = msg.y;
      nonlinz = msg.z;
      xoff = msg.xoff;
      yoff = msg.yoff;
  }

  //! Setpoint callback. If this function is called, the setpoint is changed.
  void setpointCallback(const geometry_msgs::Pose setpoint)
  {
    // Coordinate frame: absolute world frame.
    // Since a pose is used as input, we have to deal with the quaternion.
    tf::Quaternion q(setpoint.orientation.x, setpoint.orientation.y, setpoint.orientation.z, setpoint.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Check if the setpoint changed. If so, reset I action. This is the only anti wind-up reset.
    if (Setpoint.x == setpoint.position.x && Setpoint.y == setpoint.position.y) {
      isum.x = 0;
      isum.y = 0;
    }

    Setpoint.x = setpoint.position.x;
    Setpoint.y = setpoint.position.y;
    Setpoint.z = setpoint.position.z;
    Setpoint.yaw = yaw;
  }

  //! Error calculation of this class: if a new current position is received, calculate the errors
  void odomCallback(const nav_msgs::Odometry odom)
  {
    // Coordinate frame: absolute world frame.
    // Set time when message comes in (used for derivative)
    double current_time;
    current_time = ros::Time::now().toSec();
    times_.push_back(current_time);

    // Prepare variables for error filtering
    double ex, ez, ey, eyaw;

    // Calculate current rotations based on sensor data. This is important, because it is used in all position error calculations. Filter yaw!
    tf::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    yaws_.push_back(yaw);
    yaw_ = avg(yaws_,10);

    std_msgs::Float32 yaw_msg;
    yaw_msg.data = yaw_;
    yaw_publisher_.publish(yaw_msg);

    // Calculate errors.
    ex = (Setpoint.x - odom.pose.pose.position.x);
    ey = (Setpoint.y - odom.pose.pose.position.y);
    ez = (Setpoint.z - odom.pose.pose.position.z);
    eyaw = Setpoint.yaw - yaw_;

    // Store errors for later reference
    errorsx_.push_back(ex);
    errorsy_.push_back(ey);
    errorsz_.push_back(ez);
    errorsyaw_.push_back(ey);

    // Do some memory management 
    if(times_.size() > error_memory_) 
    {
      errorsx_.erase(errorsx_.begin());
      errorsy_.erase(errorsy_.begin());
      errorsz_.erase(errorsz_.begin());
      errorsyaw_.erase(errorsyaw_.begin());
      times_.erase(times_.begin());
      yaws_.erase(yaws_.begin());
    }

    // Store speeds for velocity damping
    speeds_.push_back(odom.twist.twist);
    if(speeds_.size() > 15)
    {
      speeds_.erase(speeds_.begin());
    }

    // Save definite error 
    Error.x_translational = ex; 
    Error.y_translational = ey;
    Error.z_translational = ez;
    Error.z_rotational = eyaw;

    // Save to store for I action. For some wicked reason, we have to look two frames back
    if(i_action_)
    {
        int n = times_.size();
        isum.x = isum.x + 0.5*ex*(times_[n]-times_[n-2]);
        isum.y = isum.y + 0.5*ey*(times_[n]-times_[n-2]);
    }
    else
    {
      isum.x = 0;
      isum.y = 0;
    }

    // Get derivative of the error by using a fixed order fixed window approach
    ErrorDot.x_translational = fofw(errorsx_, times_, 6);
    ErrorDot.y_translational = fofw(errorsy_, times_, 6);
    ErrorDot.z_translational = fofw(errorsz_, times_, 6);
    ErrorDot.z_rotational = fofw(errorsyaw_, times_, 10);

    std_msgs::Float32 errord_msg;
    errord_msg.data = ErrorDot.x_translational;
    errordx_publisher_.publish(errord_msg);

    // If hovermode is enabled, and you are within the treshold, publish hover message
    if(hovermode_)
    {
      if(std::abs(Error.x_translational) < hover_treshold_ && std::abs(Error.y_translational) < hover_treshold_ && std::abs(Error.z_translational) < hover_treshold_)
      {
        Error.x_translational = 0;
        Error.y_translational = 0;
        Error.z_translational = 0;
        Error.z_rotational = 0;

        ErrorDot.x_translational = 0;
        ErrorDot.y_translational = 0;
        ErrorDot.z_translational = 0;
        ErrorDot.z_rotational = 0;
        ROS_INFO("HOVERING");
      }
    }

    // Publish once every x ms, based on the frequency.
    if(current_time - previous_publish_time_ > 1/freq_)
    {
      publishControl();
      previous_publish_time_ = current_time;  
    }
    
  }
  
  void simulationCallback(const gazebo_msgs::ModelStates pose)
  {
    // This function is called whenever a message on the simulation channel is found
    // This is untested at the moment

    static int simulationDroneID;
    if (simulationDroneID == 0){
        for (int i=0;pose.name.size()-1;i++){
            if (pose.name[i] == "quadrotor"){
                simulationDroneID = i;
                break;
            }
        }
    }
    
    // Coordinate frame: absolute world frame.
    // Set time when message comes in (used for derivative)
    double current_time;
    current_time = ros::Time::now().toSec();
    times_.push_back(current_time);

    // Prepare variables for error filtering
    double ex, ez, ey, eyaw;
    
        
    // Calculate roll pitch and yaw from quaternion data
    tf::Quaternion q(pose.pose[simulationDroneID].orientation.x, pose.pose[simulationDroneID].orientation.y, pose.pose[simulationDroneID].orientation.z, pose.pose[simulationDroneID].orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);


    yaws_.push_back(yaw);
    yaw_ = avg(yaws_,10);

    std_msgs::Float32 yaw_msg;
    yaw_msg.data = yaw_;
    yaw_publisher_.publish(yaw_msg);

    
    
 // Calculate errors.
    ex = -(Setpoint.x - pose.pose[simulationDroneID].position.x);
    ey = -(Setpoint.y - pose.pose[simulationDroneID].position.y);
    ez = (Setpoint.z - pose.pose[simulationDroneID].position.z);
    eyaw = Setpoint.yaw - yaw_;

    // Store errors for later reference
    errorsx_.push_back(ex);
    errorsy_.push_back(ey);
    errorsz_.push_back(ez);
    errorsyaw_.push_back(ey);

    // Do some memory management 
    if(times_.size() > error_memory_) 
    {
      errorsx_.erase(errorsx_.begin());
      errorsy_.erase(errorsy_.begin());
      errorsz_.erase(errorsz_.begin());
      errorsyaw_.erase(errorsyaw_.begin());
      times_.erase(times_.begin());
      yaws_.erase(yaws_.begin());
    }

    // Store speeds for velocity damping
    speeds_.push_back(pose.twist[simulationDroneID]);
    if(speeds_.size() > 15)
    {
      speeds_.erase(speeds_.begin());
    }
//    ROS_INFO_STREAM(speeds_.back().linear.y);
    // Save definite error 
    Error.x_translational = ex; 
    Error.y_translational = ey;
    Error.z_translational = ez;
    Error.z_rotational = eyaw;

    // Save to store for I action. For some wicked reason, we have to look two frames back
    if(i_action_)
    {
        int n = times_.size();
        isum.x = isum.x + 0.5*ex*(times_[n]-times_[n-2]);
        isum.y = isum.y + 0.5*ey*(times_[n]-times_[n-2]);
    }
    else
    {
      isum.x = 0;
      isum.y = 0;
    }

    // Get derivative of the error by using a fixed order fixed window approach
    ErrorDot.x_translational = fofw(errorsx_, times_, 6);
    ErrorDot.y_translational = fofw(errorsy_, times_, 6);
    ErrorDot.z_translational = fofw(errorsz_, times_, 6);
    ErrorDot.z_rotational = fofw(errorsyaw_, times_, 10);
    
//    ROS_INFO_STREAM(ErrorDot.y_translational);

    std_msgs::Float32 errord_msg;
    errord_msg.data = ErrorDot.x_translational;
    errordx_publisher_.publish(errord_msg);

    // If hovermode is enabled, and you are within the treshold, publish hover message
    if(hovermode_)
    {
      if(std::abs(Error.x_translational) < hover_treshold_ && std::abs(Error.y_translational) < hover_treshold_ && std::abs(Error.z_translational) < hover_treshold_)
      {
        Error.x_translational = 0;
        Error.y_translational = 0;
        Error.z_translational = 0;
        Error.z_rotational = 0;

        ErrorDot.x_translational = 0;
        ErrorDot.y_translational = 0;
        ErrorDot.z_translational = 0;
        ErrorDot.z_rotational = 0;
        ROS_INFO("HOVERING");
      }
    }

    // Publish once every x ms, based on the frequency.
    if(current_time - previous_publish_time_ > 1/freq_)
    {
      publishControl();
      previous_publish_time_ = current_time;  
    }    

//    // Position error
//    // FIXED DEFINITION OF THE DRONE
//    Error.x_translational = -(Setpoint.x - pose.pose[simulationDroneID].position.x);
//    Error.y_translational = -(Setpoint.y - pose.pose[simulationDroneID].position.y);
//    Error.z_translational = Setpoint.z - pose.pose[simulationDroneID].position.z;

//    // Publish control
//    publishControl();
  } 

  void publishControl()
  {
    // This function relates all errors in absolute world frame to body actions.
    std_msgs::Float32 errorx_msg;
    errorx_msg.data = Error.x_translational;
    errorx_publisher_.publish(errorx_msg);

    /* Start off with nonlinear coeff on P gain
    If the error is positive, a positive nonlin* will result in additional gain.
    */
    float addgainx, addgainy, addgainz;
    addgainx = 0;
    addgainy = 0;
    addgainz = 0;

    if (Error.x_translational > 0)
    {
      addgainx = nonlinx;
    }
    else
    {
      addgainx = -nonlinx;
    }

    if (Error.y_translational > 0)
    {
      addgainy = nonliny;
    }
    else
    {
      addgainy = -nonliny;
    }

    if (Error.z_translational > 0)
    {
      addgainz = nonlinz;
    }
    else
    {
      addgainz = -nonlinz;
    }

    // P - action
    double px, py, pz, pyaw;
    px = Error.x_translational * (Gains.p_translational*(1+addgainx));
    py = Error.y_translational * (Gains.p_translational*(1+addgainy));
    pz = Error.z_translational * (Gains.p_z*(1+addgainz));
    pyaw = Error.z_rotational * Gains.p_rotational;

    std_msgs::Float32 p_action;
    p_action.data = px;
    p_publisher_.publish(p_action);

    // D - action
    double dx, dy, dz, dyaw;
    dx = ErrorDot.x_translational * Gains.d_translational;
    dy = ErrorDot.y_translational * Gains.d_translational;
    dz = ErrorDot.z_translational * Gains.d_z;
    dyaw = ErrorDot.z_rotational * Gains.d_rotational;

    std_msgs::Float32 d_action;
    d_action.data = dx;
    d_publisher_.publish(d_action);

    // Absolute velocity damping
    double vx, vy, vz;
    vx = 0; vy = 0; vz = 0;
    // Make it go through honey! 
    if(velocity_damping_)
    {
      vx = speeds_.back().linear.x * Gains.velocity;
      vy = speeds_.back().linear.y * Gains.velocity;
      vz = speeds_.back().linear.z * Gains.velocity;
    }

    std_msgs::Float32 vel_action;
    vel_action.data = vx;
    velocity_damping_publisher_.publish(vel_action);

    // If I action is present and enable
    double ix, iy;
    if(i_action_)
    {
      ix = isum.x * Gains.i;
      iy = isum.y * Gains.i;
    }
    else
    {
      ix = 0;
      iy = 0;
    }

    // Total required actions in world frame
    double wx, wy, wz, wyaw;
    wx = px + dx + vx + xoff + ix;
    wy = py + dy + vy + yoff + iy;
    wz = pz + dz + vz;
    wyaw = pyaw + dyaw;

    // Transformation to the body fixed frame
    // This can differ between simulation and real life
    double qx, qy, qz, qyaw;
    if(simulation)
    {
      qx = wx*-cos(yaw_) + wy*-sin(yaw_);
      qy = wx*sin(yaw_) + wy*-cos(yaw_);
    }
    else
    {
      qx = wx*-cos(yaw_) + wy*-sin(yaw_);
      qy = wx*sin(yaw_) + wy*-cos(yaw_);
    }
    qz = wz;
    qyaw = wyaw;
       

    velocity_.linear.x = qx;
    velocity_.linear.y = qy;
    velocity_.linear.z = qz;
    velocity_.angular.z = qyaw;

    // LIMIT output to 1
    
    double LIMIT = Gains.v_limit;
    
    if(velocity_.linear.x > LIMIT) { velocity_.linear.x = LIMIT;}
    if(velocity_.linear.y > LIMIT) { velocity_.linear.y = LIMIT;}
    if(velocity_.linear.z > LIMIT) { velocity_.linear.z = LIMIT;}
    if(velocity_.linear.x < -LIMIT) { velocity_.linear.x = -LIMIT;}
    if(velocity_.linear.y < -LIMIT) { velocity_.linear.y = -LIMIT;}
    if(velocity_.linear.z < -LIMIT) { velocity_.linear.z = -LIMIT;}

    if(velocity_.angular.x > LIMIT) { velocity_.angular.x = LIMIT;}
    if(velocity_.angular.y > LIMIT) { velocity_.angular.y = LIMIT;}
    if(velocity_.angular.z > LIMIT) { velocity_.angular.z = LIMIT;}
    if(velocity_.angular.x < -LIMIT) { velocity_.angular.x = -LIMIT;}
    if(velocity_.angular.y < -LIMIT) { velocity_.angular.y = -LIMIT;}
    if(velocity_.angular.z < -LIMIT) { velocity_.angular.z = -LIMIT;}


    double simulationOffset_x = 0;
    double simulationOffset_y = 0;
    double simulationOffset_z = 0;

    // Simulation offset
    if (simulation){
        // Offset the gain if the model is not balanced. Find it by running simulation with P-controller and set [0,0,1] as setpoint.
        // See how much it is off in which directions at which gain P. Then your offset velocity should be gain_P* (-) error_xyz.
        double heightFactor = 0;
        if (Setpoint.z < 0.3){
            heightFactor = (0.3 - Setpoint.z);
        } else {
            heightFactor = 1;
        }
        
        simulationOffset_x = -1.025*0.5 * heightFactor;     
    }
    
    velocity_.linear.x = velocity_.linear.x + simulationOffset_x;
    velocity_.linear.y = velocity_.linear.y + simulationOffset_y;
    velocity_.linear.z = velocity_.linear.z + simulationOffset_z;

    std_msgs::Float32 tot_x;
    tot_x.data = velocity_.linear.x;
    x_publisher_.publish(tot_x);
    velocity_publisher_.publish(velocity_);
  }

  //! Simple LowPassFilter implementation
  double lowPassFilter(double x, double y0, double dt, double T) // Extremely simple filter 
  {
     double res = y0 + (x - y0) * (dt/(dt+T));
     return res;
  }

  //! First order fixed window approach to get the derivative
  double fofw(std::vector<double> memory, std::vector<double> times, int samplesBack)
  {
    int n = times.size();
    if(n > samplesBack)
    {
      return (memory[n-1] - memory[n-1-samplesBack])/(times[n-1]-times[n-1-samplesBack]);
    }
    else if(n > 1)
    {
      return (memory[n-1] - memory[0])/(times[n-1]-times[0]);
    }
  }

  //! Get the average from a set of values
  double avg(std::vector<double> memory, int samplesBack)
  {
    if(memory.size() > samplesBack){
      double s;
      s = 0;
      for(int n = 0; n< samplesBack; n++)
      {
          s = s + memory[memory.size()-n-1];
      }
      return s/samplesBack;
    }
    else 
    {
      return memory.back();
    }

  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  Control control;
  ros::spin();
  return 0;
}

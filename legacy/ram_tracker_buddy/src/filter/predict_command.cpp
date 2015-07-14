#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <ros/ros.h>

/**
  * Class for prediction using the commander design pattern
  * Stores all the variables needed to revert back to the state before this prediction
  *
  * This could basically also be a struct,
  * this class can be used to add functions to be able to modify the prediction,
  * but this is not used at the moment
  *
  * May 2015 - Robin Hoogervorst - RaM 
  */
class PredictCommand {

private:


public:

  Eigen::Matrix<float, 6, 1> preState;
  Eigen::Matrix<float, 6, 6> preCovariance;
  Eigen::Vector3f world_lin_acc;
  Eigen::Matrix<float, 3, 3> lin_cov;
  double dt;
  ros::Time timeStamp;

  PredictCommand(Eigen::Matrix<float, 6, 1> state,
                Eigen::Matrix<float, 6, 6> covariance,
                Eigen::Vector3f _world_lin_acc,
                Eigen::Matrix<float, 3, 3> _lin_cov,
                double _dt,
                ros::Time _timeStamp)
  {
      preState        = state;
      preCovariance   = covariance;
      world_lin_acc   = _world_lin_acc;
      lin_cov         = _lin_cov;
      dt              = _dt;
      timeStamp       = _timeStamp;
  }
};

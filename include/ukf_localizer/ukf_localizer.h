/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <iostream>
#include <vector>
#include <chrono>

#include <ros/ros.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include "Eigen/Dense"
#include <ros/ros.h>
#include <vector>
#include <string>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
class UKFLocalizer
{
public:
  UKFLocalizer();
  ~UKFLocalizer();

private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_pose_, pub_pose_cov_, pub_twist_;
  ros::Subscriber  sub_pose_, sub_pose_with_cov_, sub_initialpose_,sub_twist_;
  ros::Timer timer_control_, timer_tf_;
  tf2_ros::TransformBroadcaster tf_br_;
  int num_state_;
  int n_aug_;
  // Dimension of laser measurement space
  int n_laser_;
  /// Sigma point spreading parameter
  double lambda_;
int ukf_rate_;
 double delta_t;
 std::string pose_frame_id_;

 //standard deviation for lidar measurement
double pose_stddev_x_ ;
double pose_stddev_y_;
double twist_stddev_x_;
double twist_stddev_z_;
// Process noise standard deviation longitudinal acceleration in m/s^2
double std_a_ ;

  // Process noise standard deviation yaw acceleration in rad/s^2
double std_yawdd_ ;
//state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::MatrixXd x_ctrv_;

// state covariance matrix
  Eigen::MatrixXd p_ctrv_;

// predicted sigma points matrix
  Eigen::MatrixXd x_sig_pred_ctrv_;

// Laser measurement covariance matrix
  MatrixXd S_laser_;

// Twist measurement covariance matrix
  MatrixXd S_twist_;

  // Cross correlation matrix for laser measurements
  MatrixXd Tc_laser_;
// Cross correlation matrix for twist measurements
  MatrixXd Tc_twist_;

  // Kalman gain for laser measurements
  MatrixXd K_laser_;
// Laser measurement noise covariance matrix
  MatrixXd R_laser_;
 // Predicted state mean in lidar measurement space
  VectorXd z_pred_laser_;

  // Utility variable for lidar update
  VectorXd deltaz_laser_;

  // predicted sigma points in lidar measurement space
  MatrixXd Zsig_laser_;

// Kalman gain for twist measurements
  MatrixXd K_twist_;
// CAN measurement noise covariance matrix
  MatrixXd R_twist_;
 // Predicted state mean in CAN measurement space
  VectorXd z_pred_twist_;

  // Utility variable for twist update
  VectorXd deltaz_twist_;

  // predicted sigma points in twist measurement space
  MatrixXd Zsig_twist_;

///* Weights of sigma points
  VectorXd weights_;
  //* Weights of sigma points
  Eigen::VectorXd weights_c_;
  Eigen::VectorXd weights_s_;

///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;
///* state covariance matrix
  MatrixXd P_;
// Augmented state vector
  VectorXd x_aug_;
  
  // Utility vector for updates
VectorXd deltax_;
VectorXd meas_;
VectorXd meas_twist_;


double NIS_laser_;
double NIS_twist_;
  // Augmented state covariance matrix
  MatrixXd P_aug_;

  // Matrix such that L_^T L_ = P_aug_; used for constructing sigma points
  MatrixXd L_;

  // Augmented sigma points matrix
  MatrixXd Xsig_aug_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;


  

  /* for model prediction */
 
  std::shared_ptr<geometry_msgs::PoseStamped> current_pose_ptr_;   //!< @brief current measured pose
  std::shared_ptr<geometry_msgs::TwistStamped> current_twist_ptr_;   //!< @brief current measured pose
  geometry_msgs::PoseStamped current_ukf_pose_;                    //!< @brief current estimated pose
  geometry_msgs::TwistStamped current_ukf_twist_;                  //!< @brief current estimated twist
  boost::array<double, 25ul> current_pose_covariance_;

 
  void timerCallback(const ros::TimerEvent &e);

  
  void timerTFCallback(const ros::TimerEvent &e);

 
  void callbackPose(const geometry_msgs::PoseStamped::ConstPtr &msg);

  void callbackTwist(const geometry_msgs::TwistStamped::ConstPtr &twist);
 
  
  void callbackPoseWithCovariance(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

  void initUKF();

  
  void predictKinematicsModel(const double dt); 
  void callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped &msg);

  
  void measurementUpdatePose(const geometry_msgs::PoseStamped &pose);
  void measurementUpdateTwist(const geometry_msgs::TwistStamped &twist);


  bool getTransformFromTF(std::string parent_frame, std::string child_frame, geometry_msgs::TransformStamped &transform);

 
  double normalizeYaw(const double &yaw);

  
  void setCurrentResult();

  
  void publishEstimatedPose();

  void showCurrentX();

};


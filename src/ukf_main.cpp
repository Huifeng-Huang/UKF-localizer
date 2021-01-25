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

#include "ukf_localizer/ukf_localizer.h"
#include <ros/console.h>

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl



using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

UKFLocalizer::UKFLocalizer() : nh_(""), pnh_("~"), n_aug_(7), num_state_(5),  pose_stddev_x_(0.1),pose_stddev_y_(0.1)
{
 // initial state vector
  x_ctrv_ = Eigen::MatrixXd(5, 1);
// initial covariance matrix
  p_ctrv_ = Eigen::MatrixXd(5, 5);
// predicted sigma points matrix
  x_sig_pred_ctrv_ = Eigen::MatrixXd(num_state_, 2 * num_state_ + 1);

Zsig_laser_ = MatrixXd( 2, 15 );
Zsig_twist_ = MatrixXd( 2, 15);
z_pred_laser_ = VectorXd(2);
z_pred_twist_ = VectorXd(2);


deltaz_laser_ = VectorXd( 2);
deltaz_twist_ = VectorXd(2);
  Zsig_laser_ = MatrixXd( 2, 15 );
  pose_stddev_x_ = 0.1; // provided by Hesai 
  pose_stddev_y_ = 0.1; // provided by Hesai
  twist_stddev_x_ =0.1;
  twist_stddev_z_ = 0.1;
 // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3.;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.;

  ukf_rate_= 50;
  delta_t = 1.0 / ukf_rate_;
  // Laser measurement noise covariance matrix is constant/persistent
  R_laser_ = MatrixXd(2, 2);
  R_laser_.fill(0.);
  R_laser_(0,0) = pose_stddev_x_*pose_stddev_x_;
  R_laser_(1,1) = pose_stddev_y_*pose_stddev_y_;
  S_laser_ = MatrixXd( 2, 2);
  Tc_laser_ = MatrixXd( 5, 2);
  K_laser_ = MatrixXd(5, 2 );


// Twist measurement noise covariance matrix is constant/persistent
  R_twist_ = MatrixXd(2, 2);
  R_twist_.fill(0.);
  R_twist_(0,0) = twist_stddev_x_*twist_stddev_x_;
  R_twist_(1,1) = twist_stddev_z_*twist_stddev_z_;
  S_twist_ = MatrixXd( 2, 2);
  Tc_twist_ = MatrixXd( 5, 2);
  K_twist_ = MatrixXd(5, 2 );
  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

 x_aug_ = VectorXd( n_aug_ );

  deltax_ = VectorXd( n_aug_ );

  Xsig_aug_ = MatrixXd( n_aug_, 2*n_aug_+1 );

  Xsig_pred_ = MatrixXd( num_state_, 2*n_aug_+1 );

  P_aug_ = MatrixXd( n_aug_, n_aug_ ); 

  L_ = MatrixXd( n_aug_, n_aug_ ); 

 
  
  // create vector for weights
  weights_c_ = Eigen::VectorXd(2 * num_state_ + 1);
  weights_s_ = Eigen::VectorXd(2 * num_state_ + 1);

  



  /* initialize ros system */
  std::string in_pose, in_pose_with_cov, in_twist, out_pose, out_twist, out_pose_with_covariance, imu,in_pose_1, in_pose_with_cov_1;
  pnh_.param("input_pose_name", in_pose, std::string("/ndt_pose"));
 pnh_.param("input_pose_name", in_pose_1, std::string("/gnss_pose"));
  //pnh_.param("input_pose_with_cov_name", in_pose_with_cov, std::string("/ndt_pose_with_covariance"));
 // pnh_.param("input_pose_with_cov_name", in_pose_with_cov_1, std::string("/gnss_pose_with_covariance"));
  pnh_.param("input_twist_name", in_twist, std::string("/can_twist"));
  pnh_.param("output_pose_name", out_pose, std::string("/ukf_pose"));
  pnh_.param("output_twist_name", out_twist, std::string("/ukf_twist"));
  pnh_.param("imu_name", imu, std::string("/imu"));
  pnh_.param("output_pose_with_covariance_name", out_pose_with_covariance, std::string("/ukf_pose_with_covariance"));
  timer_control_ = nh_.createTimer(ros::Duration(1.0 / 50.0), &UKFLocalizer::timerCallback, this);
  pnh_.param("pose_frame_id", pose_frame_id_, std::string("/map"));
  timer_tf_ = nh_.createTimer(ros::Duration(1.0 / 10.0), &UKFLocalizer::timerTFCallback, this);
  pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(out_pose, 1);
  pub_pose_cov_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(out_pose_with_covariance, 1);
  pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped>(out_twist, 1);
  sub_initialpose_ = nh_.subscribe("/initialpose", 1, &UKFLocalizer::callbackInitialPose, this);
  sub_pose_ = nh_.subscribe(in_pose, 1, &UKFLocalizer::callbackPose, this);
  sub_twist_ = nh_.subscribe(in_twist,1, &UKFLocalizer::callbackTwist, this);
  

  initUKF();

 
};

UKFLocalizer::~UKFLocalizer(){};

/*
 * timerCallback
 */
void UKFLocalizer::timerCallback(const ros::TimerEvent &e)
{
  cout << "Timer created successfully " << endl;
  /* predict model in UKF */
  auto start = std::chrono::system_clock::now();
 
  
  predictKinematicsModel(delta_t);

  

  if (current_pose_ptr_ != nullptr)
  {
    
    start = std::chrono::system_clock::now();
    measurementUpdatePose(*current_pose_ptr_);
   
  }

 if (current_twist_ptr_ != nullptr)
  {
    
    start = std::chrono::system_clock::now();
    measurementUpdateTwist(*current_twist_ptr_);
   
  }
 
  

  /* set current pose, twist */
  setCurrentResult();

  publishEstimatedPose();
}



/*
 * setCurrentResult
 */
void UKFLocalizer::setCurrentResult()
{ cout << "Fetching current result " << endl;
  current_ukf_pose_.header.frame_id = pose_frame_id_;
  current_ukf_pose_.header.stamp = ros::Time::now();
  current_ukf_pose_.pose.position.x = x_(0);
  current_ukf_pose_.pose.position.y = x_(1);

  tf2::Quaternion q_tf;
  double roll, pitch, yaw;
  if (current_pose_ptr_ != nullptr)
  {
    current_ukf_pose_.pose.position.z = current_pose_ptr_->pose.position.z;
    tf2::fromMsg(current_pose_ptr_->pose.orientation, q_tf); /* use Pose pitch and roll */
    tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);
  }
  else
  {
    current_ukf_pose_.pose.position.z = 0.0;
    roll = 0;
    pitch = 0;
  }
  yaw = x_(3);
  q_tf.setRPY(roll, pitch, yaw);
  tf2::convert(q_tf, current_ukf_pose_.pose.orientation);

  current_ukf_twist_.header.frame_id = "base_link";
  current_ukf_twist_.header.stamp = ros::Time::now();
  current_ukf_twist_.twist.linear.x = x_(2);
current_ukf_twist_.twist.linear.y = 0;
current_ukf_twist_.twist.linear.z = 0;
  current_ukf_twist_.twist.angular.z = x_(4);
current_ukf_twist_.twist.angular.x = 0;
current_ukf_twist_.twist.angular.y = 0;
}

/*
 * timerTFCallback
 */
void UKFLocalizer::timerTFCallback(const ros::TimerEvent &e)
{
  if (current_ukf_pose_.header.frame_id == "")
    return;

  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = current_ukf_pose_.header.frame_id;
  transformStamped.child_frame_id = "ukf_pose";
  transformStamped.transform.translation.x = current_ukf_pose_.pose.position.x;
  transformStamped.transform.translation.y = current_ukf_pose_.pose.position.y;
  transformStamped.transform.translation.z = current_ukf_pose_.pose.position.z;

  transformStamped.transform.rotation.x = current_ukf_pose_.pose.orientation.x;
  transformStamped.transform.rotation.y = current_ukf_pose_.pose.orientation.y;
  transformStamped.transform.rotation.z = current_ukf_pose_.pose.orientation.z;
  transformStamped.transform.rotation.w = current_ukf_pose_.pose.orientation.w;

  tf_br_.sendTransform(transformStamped);

}

/*
 * getTransformFromTF
 */
bool UKFLocalizer::getTransformFromTF(std::string parent_frame, std::string child_frame, geometry_msgs::TransformStamped &transform)
{
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  ros::Duration(0.1).sleep();
  if (parent_frame.front() == '/')
    parent_frame.erase(0, 1);
  if (child_frame.front() == '/')
    child_frame.erase(0, 1);

  for (int i = 0; i < 50; ++i)
  {
    try
    {
      transform = tf_buffer.lookupTransform(parent_frame, child_frame, ros::Time(0));
      return true;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(0.1).sleep();
    }
  }
  return false;
}



/*
 * callbackInitialPose
 */
void UKFLocalizer::callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped &initialpose)
{
  geometry_msgs::TransformStamped transform;
  if (!getTransformFromTF(pose_frame_id_, initialpose.header.frame_id, transform))
  {
    ROS_ERROR("[UKF] TF transform failed. parent = %s, child = %s", pose_frame_id_.c_str(), initialpose.header.frame_id.c_str());
  };

  

  x_(0) = initialpose.pose.pose.position.x + transform.transform.translation.x;
  x_(1) = initialpose.pose.pose.position.y + transform.transform.translation.y;
 x_(2) = tf2::getYaw(initialpose.pose.pose.orientation) + tf2::getYaw(transform.transform.rotation);
 
   x_(3) = 0.0;
   x_(4) = 0.0;

  P_(0,0) = initialpose.pose.covariance[0];
  P_(1,1) = initialpose.pose.covariance[6 + 1];
  P_(2, 2) = initialpose.pose.covariance[6 * 5 + 5];

  P_(3, 3) = 0.01;
  P_(4, 4) = 0.01;

  
};
/*imu
 * callbackPose
 */
void UKFLocalizer::callbackPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
 
  
    current_pose_ptr_ = std::make_shared<geometry_msgs::PoseStamped>(*msg);
  
};



/*
 * callbackPoseWithCovariance
 */



/*
 * callbackTwist
 */

void UKFLocalizer::callbackTwist(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
 
  
    current_twist_ptr_ = std::make_shared<geometry_msgs::TwistStamped>(*msg);
  
};

/*
 * initUKF
 */
void UKFLocalizer::initUKF()
{cout << "Initializing unscented Kalman filter" << endl;
 x_ << 0.,0., 0.,0.,0.;

 P_.fill(0.);
    P_(0,0) = 1.;
    P_(1,1) = 1.;
    P_(2,2) = 1.;
    P_(3,3) = 1.;
    P_(4,4) = 1.;

weights_ = VectorXd( 2*n_aug_+1 );
 weights_(0) = lambda_/( lambda_ + n_aug_ );
  for( int i=1; i<2*n_aug_+1; i++ )  
     weights_(i) = 0.5/( n_aug_ + lambda_ );
 
 
}

/*
 * predictKinematicsModel
 */
void UKFLocalizer::predictKinematicsModel(const double dt) {
 cout << "Start predicting Kinematic Model " << endl;
     //create augmented mean state
  x_aug_.head(5) = x_;
  x_aug_(5) = 0;
  x_aug_(6) = 0;
 
  //create augmented covariance matrix
  P_aug_.fill(0.);
  P_aug_.topLeftCorner(5,5) = P_;
  P_aug_(5,5) = std_a_*std_a_;
  P_aug_(6,6) = std_yawdd_*std_yawdd_;
  
  //create square root matrix
  L_ = P_aug_.llt().matrixL();
  
  //create augmented sigma points
  Xsig_aug_.col(0) = x_aug_;
  for( int i = 0; i < n_aug_; i++)
  {
    Xsig_aug_.col(i+1)       = x_aug_ + sqrt(lambda_+n_aug_)*L_.col(i);
    Xsig_aug_.col(i+n_aug_+1) = x_aug_ - sqrt(lambda_+n_aug_)*L_.col(i);
  }

  // Run each augmented sigma points through the process model with noise
  // (prediction step)
 
  double dt2 = delta_t*delta_t;
  for( int pt = 0; pt < 2*n_aug_+1; pt++ )
  {
      double x       = Xsig_aug_(0,pt);
      double y       = Xsig_aug_(1,pt);
      double v       = Xsig_aug_(2,pt);
      double psi     = Xsig_aug_(3,pt);
      double psid    = Xsig_aug_(4,pt);
      double nua     = Xsig_aug_(5,pt);
      double nupsidd = Xsig_aug_(6,pt);
      
      if( psid == 0. )
      {
        Xsig_pred_(0,pt) = x + v*cos(psi)*dt + 0.5f*dt2*cos(psi)*nua;
        Xsig_pred_(1,pt) = y + v*sin(psi)*dt + 0.5f*dt2*sin(psi)*nua;
      }
      else
      {
        Xsig_pred_(0,pt) = x + ( v/psid )*( sin(psi + psid*dt) - sin(psi) ) 
                          + 0.5f*dt2*cos(psi)*nua;
        Xsig_pred_(1,pt) = y + ( v/psid )*( -cos(psi + psid*dt) + cos(psi) ) 
                          + 0.5f*dt2*sin(psi)*nua;
      }    
      Xsig_pred_(2,pt) = v    +           dt*nua;
      Xsig_pred_(3,pt) = psi  + psid*dt + 0.5f*dt2*nupsidd;
      Xsig_pred_(4,pt) = psid +           dt*nupsidd;
  }

  //predict state mean
  x_.fill(0.);
  for( int i = 0; i < 2*n_aug_+1; i++ )
    x_ = x_ + weights_(i)*Xsig_pred_.col(i);

  //predict state covariance matrix
  P_.fill(0.);
  for( int i = 0; i < 2*n_aug_+1; i++)
  {
    deltax_ = Xsig_pred_.col(i) - x_;
    while( deltax_(3) > M_PI ) deltax_(3) -= 2.*M_PI;
    while( deltax_(3) < -M_PI ) deltax_(3) += 2.*M_PI;
    // outer product
    // https://eigen.tuxfamily.org/dox/group__QuickRefPage.html#matrixonly
    P_ = P_ + weights_(i)*deltax_*deltax_.transpose();   
   
   }


 }

/*
 * measurementUpdatePose
 */
void UKFLocalizer::measurementUpdatePose(const geometry_msgs::PoseStamped &pose)
{
  // Transform sigma points into measurement space. The shortcut is to take the sigma points from the prediction part into the measurement model
  for( int pt = 0; pt < 2*n_aug_ + 1; pt++ )
  {
    Zsig_laser_(0,pt) =  Xsig_pred_(0,pt);
    Zsig_laser_(1,pt) =  Xsig_pred_(1,pt);
  }
  
  //calculate mean predicted measurement
  z_pred_laser_.fill(0.);
  for( int pt = 0; pt < 2*n_aug_ + 1; pt++ )
    z_pred_laser_ = z_pred_laser_ + weights_(pt)*Zsig_laser_.col(pt);
  
  //calculate measurement covariance matrix S_laser_
  S_laser_.fill(0.);
  for( int pt = 0; pt < 2*n_aug_ + 1; pt++ )
  {
    deltaz_laser_ = Zsig_laser_.col(pt) - z_pred_laser_;
    S_laser_ = S_laser_ + weights_(pt)*deltaz_laser_*deltaz_laser_.transpose();
  }

  S_laser_ = S_laser_ + R_laser_;

  //calculate cross correlation matrix
  Tc_laser_.fill(0.);



  /* Set measurement matrix */
  //Eigen::MatrixXd x_(5, 1);
  //x_ << pose.pose.position.x, pose.pose.position.y, 0, 0.,0.,  0.;



  for( int pt = 0; pt < 2*n_aug_ + 1; pt++ )
  {
      deltax_ =  Xsig_pred_.col(pt) - x_;
      deltaz_laser_ = Zsig_laser_.col(pt) - z_pred_laser_;
      Tc_laser_ = Tc_laser_ + weights_(pt)*deltax_*deltaz_laser_.transpose();
  }
  
  //calculate K_laser_alman gain K_laser_;
  K_laser_ = Tc_laser_*S_laser_.inverse();
  meas_ = Eigen::VectorXd(2);
 meas_ << pose.pose.position.x, pose.pose.position.y;
      //meas_vec.push_back(meas);
  //update state mean and covariance matrix
  deltaz_laser_ = meas_ - z_pred_laser_;
  
  x_ = x_ + K_laser_*deltaz_laser_;
  P_ = P_ - K_laser_*S_laser_*K_laser_.transpose();

// Uncomment the following to print normalized innovation squared (NIS_laser_),
  // so that it can be plotted and serve as a consistency check on 
  // our choice of process noise values
  NIS_laser_ = deltaz_laser_.transpose()*S_laser_.inverse()*deltaz_laser_;
  cout << "NIS_Laser is :" << NIS_laser_ << endl;
}

void UKFLocalizer::measurementUpdateTwist(const geometry_msgs::TwistStamped &twist)
{
  // Transform sigma points into measurement space. The shortcut is to take the sigma points from the prediction part into the measurement model
  for( int pt = 0; pt < 2*n_aug_ + 1; pt++ )
  {
    Zsig_twist_(0,pt) =  Xsig_pred_(2,pt);
    Zsig_twist_(1,pt) =  Xsig_pred_(4,pt);
  }
  
  //calculate mean predicted measurement
  z_pred_twist_.fill(0.);
  for( int pt = 0; pt < 2*n_aug_ + 1; pt++ )
    z_pred_twist_ = z_pred_twist_ + weights_(pt)*Zsig_twist_.col(pt);
  
  //calculate measurement covariance matrix S_laser_
  S_twist_.fill(0.);
  for( int pt = 0; pt < 2*n_aug_ + 1; pt++ )
  {
    deltaz_twist_ = Zsig_twist_.col(pt) - z_pred_twist_;
    S_twist_ = S_twist_ + weights_(pt)*deltaz_twist_*deltaz_twist_.transpose();
  }

  S_twist_ = S_twist_ + R_twist_;

  //calculate cross correlation matrix
  Tc_twist_.fill(0.);



  /* Set measurement matrix */
  //Eigen::MatrixXd x_(5, 1);
  //x_ << pose.pose.position.x, pose.pose.position.y, 0, 0.,0.,  0.;



  for( int pt = 0; pt < 2*n_aug_ + 1; pt++ )
  {
      deltax_ =  Xsig_pred_.col(pt) - x_;
      deltaz_twist_ = Zsig_twist_.col(pt) - z_pred_twist_;
      Tc_twist_ = Tc_twist_ + weights_(pt)*deltax_*deltaz_twist_.transpose();
  }
  
  //calculate K_laser_alman gain K_laser_;
  K_twist_ = Tc_twist_*S_twist_.inverse();
  meas_twist_ = Eigen::VectorXd(2);
 meas_twist_<< twist.twist.linear.x, twist.twist.angular.z;
      //meas_vec.push_back(meas);
  //update state mean and covariance matrix
  deltaz_twist_ = meas_twist_ - z_pred_twist_;
  
  x_ = x_ + K_twist_*deltaz_twist_;
  P_ = P_ - K_twist_*S_twist_*K_twist_.transpose();

// Uncomment the following to print normalized innovation squared (NIS_laser_),
  // so that it can be plotted and serve as a consistency check on 
  // our choice of process noise values
  NIS_twist_ = deltaz_twist_.transpose()*S_twist_.inverse()*deltaz_twist_;
  cout << "NIS_twist is :" << NIS_twist_ << endl;
}



double UKFLocalizer::normalizeYaw(const double &yaw)
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}
/*
 * publishEstimatedPose
 */
void UKFLocalizer::publishEstimatedPose()
{cout << "Publishing UKF topics" << endl;
  ros::Time current_time = ros::Time::now();
 
  

  /* publish latest pose */
  pub_pose_.publish(current_ukf_pose_);

  /* publish latest pose with covariance */
  geometry_msgs::PoseWithCovarianceStamped pose_cov;
  pose_cov.header.stamp = current_time;
  pose_cov.header.frame_id = current_ukf_pose_.header.frame_id;
  pose_cov.pose.pose = current_ukf_pose_.pose;
  pose_cov.pose.covariance[0] = P_(0,0);   // x, x
  pose_cov.pose.covariance[1] = P_(0,1);       // x, y
  pose_cov.pose.covariance[5] = P_(0,3);     // x, yaw
  pose_cov.pose.covariance[6] = P_(1,0);     // y, x
  pose_cov.pose.covariance[7] = P_(1,1);    // y, y
  pose_cov.pose.covariance[11] = P_(1,3);    // y, yaw
  pose_cov.pose.covariance[30] = P_(3,0); ;   // yaw, x
  pose_cov.pose.covariance[31] = P_(3,1);  // yaw, y
  pose_cov.pose.covariance[35] =P_(3,3); // yaw, yaw
  pub_pose_cov_.publish(pose_cov);
/* publish latest twist */
  pub_twist_.publish(current_ukf_twist_);

  /* publish yaw bias */
  

  
 }






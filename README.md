# Overview
This package generates robust less-noisy self_pose/twist integrating pose and twist messages with vehicle model. By default, this package is applicable when `/ndt_pose` and `/can_twist` are published. (`/can_twist` is converted from `/vehicle_status` by `vehicle_status_converter` in twist_generator package.) The `ukf_localizer` generates estimated pose `/ukf_pose`. 




# Input and Output
- input
    - geometry_msgs::PoseStamped ego_pose (default: `/ndt_pose`)
    - geometry_msgs::TwistStamped ego_twist (default: `/can_twist`))
    
- output
    - tf base_link : broadcasted with *tf_rate* [Hz]
    - estimated pose (geometry_msgs::PoseStamped) : filtered pose
    - estimated twist (geometry_msgs::TwistStamped) : filtered twist


# Functions


## in timer callback with *predict_frequency*
Calculate *predict* and *update* at constant intervals in timer callback. The purpose of perform *update* at timer callback is to avoid large position (or velocity) changes due to low period measurement, such as NDT matching. If the callback period is faster than the sensor period, the same sensor value is used for update. Note that since the algorithm can deal with sensor time delay, there is no lack of consistency in using the same sensor data. 


## measurement data callback

The subscribed data is saved and used for *update* of kalman filter in timer callback. T


# Parameter description

they are set in `launch/ukf_localizer.launch` 


## for Node

|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|show_debug_info|bool|display debug info|false|
|predict_frequency|double|filtering and publish frequency [Hz]|50.0|
|tf_rate|double|frqcuency for tf broadcasting [Hz]|10.0|
|extend_state_step|int|max delay step which can be dealt with in EKF. Large number increases computational cost. |50|
|enable_yaw_bias_estimation| bool |enable yaw bias estimation for LiDAR mount error|true|

## for pose measurement

|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|pose_additional_delay|double|Additional delay time for pose measurement [s]|0.0|
|pose_measure_uncertainty_time|double|Used for covariance calculation [s]|0.01|
|pose_rate|double|used for covariance calculation [Hz]|10.0|
|pose_gate_dist|double|limit of Mahalanobis distance used for outliers detection|10000.0|
|pose_stddev_x|double|standard deviation for pose position x [m]|0.05|
|pose_stddev_y|double|standard deviation for pose position y [m]|0.05|
|pose_stddev_yaw|double|standard deviation for pose yaw angle [rad]|0.025|
|use_pose_with_covariance|bool|use covariance in pose_with_covarianve message|false|

## for twist measurement
|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|twist_additional_delay|double|Additional delay time for twist [s]|0.0|
|twist_rate|double|used for covariance calculation [Hz]|10.0|
|twist_gate_dist|double|limit of Mahalanobis distance used for outliers detection|10000.0|
|twist_stddev_vx|double|standard deviation for twist linear x [m/s] |0.2|
|twist_stddev_wz|double|standard deviation for twist angular z [rad/s] |0.03|

## for process noise
|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|proc_stddev_vx_c|double|standard deviation of process noise in time differentiation expression of linear velocity x, noise for d_vx = 0|2.0|
|proc_stddev_wz_c|double|standard deviation of process noise in time differentiation expression of angular velocity z, noise for d_wz = 0|0.2|
|proc_stddev_yaw_c|double|standard deviation of process noise in time differentiation expression of yaw, noise for d_yaw = omege |0.005|
|proc_stddev_yaw_bias_c|double|standard deviation of process noise in time differentiation expression of yaw_bias, noise for d_yaw_bias = 0|0.001|

note: process noise for position x & y are calculated automatically dealing with nonlinear equation.



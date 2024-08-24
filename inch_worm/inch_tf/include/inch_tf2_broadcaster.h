#ifndef TF_BROADCASTER_H_
#define TF_BROADCASTER_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <inch_toolbox/inch_misc.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_srvs/Empty.h>


#define PI 3.14159256359

class TFBroadcaster : public inch::InchMisc
{
 public:
  TFBroadcaster();
  ~TFBroadcaster();

  /*****************************************************************************
  ** Define variables
  *****************************************************************************/
  // For time loop (count)
  double time_i = 0;
  double time_f = 0;
  double time_loop = 0;
  bool gimbal_Flag = false;

  geometry_msgs::Twist inchBase;

  double roll;
  double pitch;
  double yaw;

  /*****************************************************************************
  ** Define functions
  *****************************************************************************/

 private:
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle node_handle_;

  /*****************************************************************************
  ** ROS Parameters
  *****************************************************************************/
  std::string robot_name_;

  /*****************************************************************************
  ** Class inst
  *****************************************************************************/
  InchMisc *inch_roll_lpf;

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initSubscriber();
  void initPublisher();

  /*****************************************************************************
  ** ROS Publishers
  *****************************************************************************/
  ros::Publisher inchBase_pub_;
  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  ros::Subscriber inch_EE_meas_sub_;
  ros::Subscriber inch_EE_ref_sub_;
  ros::Subscriber inch_EE_cmd_sub_;
  ros::Subscriber inch_BasePlate_sub_;
  ros::Subscriber sbus_sub_;

  // ros::ServiceServer inch_gimbal_Flag_server_;
  
  /*****************************************************************************
  ** Define variables
  *****************************************************************************/


  /*****************************************************************************
  ** Define functions
  *****************************************************************************/

  void inch_EE_meas_callback(const geometry_msgs::Twist& msg);
  void inch_EE_ref_callback(const geometry_msgs::Twist& msg);
  void inch_EE_cmd_callback(const geometry_msgs::Twist& msg);
  void sbus_callback(const std_msgs::Int16MultiArray::ConstPtr& msg);
  void inch_Palletrone_callback(const geometry_msgs::PoseStamped& msg);
  // bool gimbal_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  
};

#endif //TF_BROADCASTER_H_

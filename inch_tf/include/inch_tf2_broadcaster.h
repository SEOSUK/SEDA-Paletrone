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
//  ros::Publisher Link1_pub_;
  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  ros::Subscriber palletrone_optitrack_sub_;

  /*****************************************************************************
  ** Define variables
  *****************************************************************************/
  //geometry_msgs::Vector3 Link1_msg;

  /*****************************************************************************
  ** Define functions
  *****************************************************************************/

  void palletroneOptitrackCallback(const geometry_msgs::PoseStamped& msg);
};

#endif //TF_BROADCASTER_H_
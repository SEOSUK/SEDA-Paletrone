#ifndef INCH_JOINT_H_
#define INCH_JOINT_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include <inch_toolbox/inch_misc.h>
#include <iostream>
#include <cmath>
#include <functional>

#define PI 3.141592

using namespace inch;

class InchJoint : public inch::InchMisc
{
 public:
  InchJoint();
  ~InchJoint();

  /*****************************************************************************
  ** Define functions
  *****************************************************************************/
  void test();


  InchMisc *inch_q1_dot_;
  InchMisc *inch_q2_dot_;
  InchMisc *inch_q1_ddot_;
  InchMisc *inch_q2_ddot_;
  InchMisc *inch_phi1_dot_;
  InchMisc *inch_phi2_dot_;
  /*****************************************************************************
  ** Define variables
  *****************************************************************************/

  Eigen::Vector2d theta_meas;
  Eigen::Vector2d phi_meas;
  Eigen::Vector2d q_meas;
  Eigen::Vector2d tau_ext;

  Eigen::Vector2d k_sp;
  Eigen::Vector2d tau_phi;
  Eigen::Vector2d tau_MCG;


  Eigen::Vector2d q_dot_meas;
  Eigen::Vector2d q_ddot_meas;
  Eigen::Vector2d phi_dot_meas;


 private:
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  /*****************************************************************************
  ** ROS Subscribers
  *****************************************************************************/
  ros::Subscriber dynamixel_callback_sub_;
  ros::Subscriber encoder_phi_callback_sub_;

  /*****************************************************************************
  ** ROS Timer
  *****************************************************************************/
  ros::Timer calc_angle_timer_;


  /*****************************************************************************
  ** Define variables
  *****************************************************************************/


  /*****************************************************************************
  ** Define functions
  *****************************************************************************/
  void dynamixel_callback(const sensor_msgs::JointState::ConstPtr &msg);
  void encoder_phi_callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
  void calc_angle_timer_callback(const ros::TimerEvent&);
  Eigen::Matrix2d M_Matrix();
  Eigen::Matrix2d C_Matrix();
  Eigen::Vector2d G_Matrix();
  Eigen::Vector2d calc_MCGDynamics();
  
  void initPublisher();
  void initSubscriber();
  void initTimerCallback();
  void initParameters();

  double Link1_length;
  double Link2_length;

  double Link1_COM;
  double Link2_COM;
  
  double Link1_mass;
  double Link2_mass;
};

#endif /*INCH_JOINT_H_*/

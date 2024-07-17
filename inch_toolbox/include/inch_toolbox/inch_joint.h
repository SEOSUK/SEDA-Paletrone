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
  void dynamixel_callback(const sensor_msgs::JointState::ConstPtr &msg);
  void encoder_phi_callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
  void calc_angle_timer_callback(const ros::TimerEvent&);

  Eigen::Vector2d MPC_controller_2Link(Eigen::Vector2d ref_, double time_loop_);
  void init_MPC_controller_2Link(double w0_Link1, double zeta_Link1, double w0_Link2, double zeta_Link2);
  void init_Link1_MPC_controller(double w0_, double zeta_);


  Eigen::Matrix2d M_Matrix();
  Eigen::Vector2d C_Matrix();
  Eigen::Vector2d G_Matrix();
  Eigen::Vector2d calc_MCGDynamics();
  
  void initPublisher();
  void initSubscriber();
  void initTimerCallback();
  void initParameters();





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

  Eigen::Vector2d tau_MPC;

  Eigen::Vector2d phi_MPC;
  Eigen::Vector2d theta_MPC_i;


  Eigen::Vector2d q_dot_meas;
  Eigen::Vector2d q_ddot_meas;
  Eigen::Vector2d phi_dot_meas;

  double Link1_length;
  double Link2_length;

  double Link1_COM;
  double Link2_COM;

  double Link1_mass;
  double Link2_mass;

  double Link1_spring;
  double Link2_spring;

  double Gravity;
  double N1;
  double N2;
  double N3;


  Eigen::Vector2d phi_offset;


  Eigen::Matrix2d K1;
  Eigen::Matrix2d K2;
  Eigen::Matrix2d K3;  
  Eigen::Matrix2d damping_coef;

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


};

#endif /*INCH_JOINT_H_*/

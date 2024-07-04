#ifndef INCH_JOINT_H_
#define INCH_JOINT_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#define PI 3.141592

class InchJoint
{
 public:
  InchJoint();
  ~InchJoint();

  /*****************************************************************************
  ** Define functions
  *****************************************************************************/
  void test();

  /*****************************************************************************
  ** Define variables
  *****************************************************************************/

  Eigen::Vector2d theta_meas;
  Eigen::Vector2d phi_meas;
  Eigen::Vector2d q_meas;
  Eigen::Vector2d tau_ext;

  Eigen::Vector2d k_sp;
  Eigen::Vector2d tau_phi;

  // From Launch File
  //Link Param
  double length_1;
  double length_2;
  double length_3;
  double com_1;
  double com_2;
  double mass_1;
  double mass_2;
  double g;
  std::string robot_name_;

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
  
  void initPublisher();
  void initSubscriber();
  void initTimerCallback();
  void initParameters();

  Eigen::Vector2d tau_ext_processing(Eigen::Vector2d q_meas_, Eigen::Vector2d tau_phi_);

};

#endif /*INCH_JOINT_H_*/

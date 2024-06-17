#ifndef INCH_MANIPULATOR_CONTROL_H_
#define INCH_MANIPULATOR_CONTROL_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/TFMessage.h>
#include <inch_toolbox/inch_workbench.h>
#include <inch_toolbox/inch_joint.h>
#include <inch_toolbox/inch_misc.h>
#define PI 3.141592

using namespace inch;

class InchControl : public inch::InchWorkbench
{
 public:
  InchControl();
  ~InchControl();

  /*****************************************************************************
  ** Define variables
  *****************************************************************************/
  // For time loop (count)
  double time_i = 0;
  double time_f = 0;
  double time_loop = 0;
  double time_real = 0;

  // From Launch File
    //Link Param
  double length_1;
  double length_2;
  double length_3;
  std::string robot_name_;

  /*****************************************************************************
  ** Define functions
  *****************************************************************************/
  void PublishData();
  void deleteToolbox();
  void SolveInverseForwardKinematics();
  
  void TimeCount();
  void Test_trajectory_generator_2dof();
  void Trajectory_mode();
  void trajectory_gimbaling();




 private:
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  /*****************************************************************************
  ** ROS Parameters
  *****************************************************************************/

  /*****************************************************************************
  ** Inchtoolbox
  *****************************************************************************/
  // Inch
  InchJoint *inch_jnt1_;
  InchJoint *inch_jnt2_;
  InchJoint *inch_jnt3_;

  InchMisc *inch_EE_1_misc_;
  InchMisc *inch_EE_2_misc_;
  
  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initPublisher();
  void initSubscriber();
  void initServer();
  
  /*****************************************************************************
  ** ROS Publishers
  *****************************************************************************/
  ros::Publisher theta_command_pub_;
  ros::Publisher EE_meas_pub_;
  ros::Publisher test_pub_;

  /*****************************************************************************
  ** ROS Subscribers
  *****************************************************************************/


  /*****************************************************************************
  ** ROS Services Clients
  *****************************************************************************/


  /*****************************************************************************
  ** Define variables
  *****************************************************************************/
  Eigen::Vector2d theta_ref;
  Eigen::Vector2d EE_meas;
  Eigen::Vector2d EE_cmd;



  std_msgs::Float64MultiArray test_msg;
  geometry_msgs::Twist EE_meas_msg;
  sensor_msgs::JointState theta_command_msg;

};

#endif //INCH_MANIPULATOR_CONTROL_H_

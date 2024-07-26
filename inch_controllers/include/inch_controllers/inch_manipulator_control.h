#ifndef INCH_MANIPULATOR_CONTROL_H_
#define INCH_MANIPULATOR_CONTROL_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
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
#include <std_msgs/Int16MultiArray.h>
#include <inch_controllers/FextYFilter.h>
#include <inch_controllers/FextZFilter.h>
#include <inch_controllers/admittance.h>
#include <std_srvs/Empty.h>
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
  double Link1_length;
  double Link2_length;

  double Link1_COM;
  double Link2_COM;

  double Link1_mass;
  double Link2_mass;

  double Link1_spring;
  double Link2_spring;

  double init_Y;
  double init_Z;

  double Gravity;
  double N1;
  double N2;
  double N3;


  double tanh_COF_;
  double deadzone_max;
  double deadzone_min;


  bool gimbal_Flag = false;
  bool stop_Flag = false;
  bool initPoseFlag;


  double tanh_COF_Y;
  double deadzone_Y_max;
  double deadzone_Y_min;

  double tanh_COF_Z;
  double deadzone_Z_max;
  double deadzone_Z_min;

  double bandpass_Y_wl;
  double bandpass_Y_wh;
  double bandpass_Z_wl;
  double bandpass_Z_wh;


  double Link1_init_phi;
  double Link2_init_phi;
  Eigen::Vector2d phi_init;
  Eigen::Vector2d tau_offset;

  std::string robot_name_;

  /*****************************************************************************
  ** Define functions
  *****************************************************************************/
  void PublishData();
  void deleteToolbox();
  void SolveInverseForwardKinematics();
  
  void TimeCount();
  void stop_Function();
  void Test_trajectory_generator_2dof();
  void Trajectory_mode();
  void trajectory_gimbaling();
  void Experiment_0623_1Link();
  void F_ext_processing();
  void init_pose_function();
  void sbus_callback(const std_msgs::Int16MultiArray::ConstPtr& msg);
  //ROS
  void inch_gimbal_EE_ref_callback(const geometry_msgs::Twist& msg);
  //bool gimbal_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool FextY_callback(inch_controllers::FextYFilter::Request& req, inch_controllers::FextYFilter::Response& res);
  bool FextZ_callback(inch_controllers::FextZFilter::Request& req, inch_controllers::FextZFilter::Response& res);
  bool admittance_callback(inch_controllers::admittance::Request& req, inch_controllers::admittance::Response& res);

  //TeamWork!
  void YujinWhile();
  void HanryungWhile();
  void SeukWhile();

  void YujinInit();
  void HanryungInit();
  void SeukInit();


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
  InchJoint *inch_joint;

  InchMisc *inch_butterworth_F_ext_y;
  InchMisc *inch_butterworth_F_ext_z;
  InchMisc *inch_link1_PID;
  InchMisc *inch_link2_PID;

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
  ros::Publisher EE_ref_pub_;
  ros::Publisher EE_cmd_pub_;
  ros::Publisher test_pub_;
  ros::Publisher F_ext_pub_;
  ros::Publisher F_ext_raw_pub_;
  ros::Publisher q_ref_pub_;
  ros::Publisher q_meas_pub_;
  ros::Publisher phi_meas_pub_;
  ros::Publisher theta_cmd_pub_;
  ros::Publisher tau_ext_pub_;

  /*****************************************************************************
  ** ROS Subscribers
  *****************************************************************************/
  ros::Subscriber dynamixel_workbench_sub_;
  ros::Subscriber Optitrack_sub_;
  ros::Subscriber sbus_sub_;
  ros::Subscriber gimbal_EE_ref_sub_;
  // ros::ServiceServer inch_gimbal_Flag_server_;
  ros::ServiceServer FextY_server;
  ros::ServiceServer FextZ_server;
  ros::ServiceServer admittance_server;
  /*****************************************************************************
  ** ROS Services Clients
  *****************************************************************************/
 

  /*****************************************************************************
  ** Define variables
  *****************************************************************************/
  Eigen::Vector2d q_ref;
  Eigen::Vector2d q_cmd;
  Eigen::Vector2d q_des;
  Eigen::Vector2d theta_ref; // theta 레퍼런스
  Eigen::Vector2d theta_des; // theta 레퍼런스 (속도리미트 걸림)
  Eigen::Vector2d theta_cmd; // theta 커맨드 (제어기 통과)
  Eigen::Vector2d EE_meas;
  Eigen::Vector2d EE_cmd;
  Eigen::Vector2d EE_gimbal_ref;
  Eigen::Vector2d EE_ref;
  Eigen::Vector2d F_ext;
  Eigen::Vector2d F_ext_;
  Eigen::Vector2d F_ext_raw;
  Eigen::Vector2d tau_ext;
  Eigen::Vector2d init_pose;
  Eigen::Vector2d init_theta;
  //Experiment_0623_1Link

  std_msgs::Float64MultiArray test_msg;
  geometry_msgs::Twist EE_meas_msg;
  geometry_msgs::Twist EE_ref_msg;
  geometry_msgs::Twist EE_cmd_msg;
  sensor_msgs::JointState theta_command_msg;
  geometry_msgs::Vector3 F_ext_msg;
  geometry_msgs::Vector3 F_ext_raw_msg;  

  geometry_msgs::Vector3 q_ref_msg;  
  geometry_msgs::Vector3 q_meas_msg;  
  geometry_msgs::Vector3 phi_meas_msg;  
  geometry_msgs::Vector3 theta_cmd_msg; 
  geometry_msgs::Vector3 tau_ext_msg; 


};

#endif //INCH_MANIPULATOR_CONTROL_H_

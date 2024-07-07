#include "inch_toolbox/inch_joint.h"

InchJoint::InchJoint()
: nh_(""), priv_nh_("~")
{

  Link1_length = priv_nh_.param<double>("Link1_length", 0);
  Link2_length = priv_nh_.param<double>("Link2_length", 0);

  Link1_COM = priv_nh_.param<double>("Link1_COM", 0);
  Link2_COM = priv_nh_.param<double>("Link2_COM", 0);

  Link1_mass = priv_nh_.param<double>("Link1_mass", 0);
  Link2_mass = priv_nh_.param<double>("Link2_mass", 0);


  initPublisher();
  initSubscriber();
  initTimerCallback();

  inch_q1_dot_ = new InchMisc(); 
  inch_q2_dot_ = new InchMisc(); 

  inch_q1_ddot_ = new InchMisc(); 
  inch_q2_ddot_ = new InchMisc(); 
  inch_phi1_dot_ = new InchMisc(); 
  inch_phi2_dot_ = new InchMisc(); 

}

InchJoint::~InchJoint()
{
  ROS_INFO("Bye InchJoint!");
  ros::shutdown();
}

void InchJoint::initPublisher()
{
  //theta_command_pub_ = node_handle_.advertise<sensor_msgs::JointState>("goal_dynamixel_position", 10); // directly to dynamixel
  
}

void InchJoint::initSubscriber()
{
  dynamixel_callback_sub_ = nh_.subscribe("/inch/joint_states", 10, &InchJoint::dynamixel_callback, this, ros::TransportHints().tcpNoDelay());
  encoder_phi_callback_sub_ = nh_.subscribe("/inch/phi_encoder", 10, &InchJoint::encoder_phi_callback, this, ros::TransportHints().tcpNoDelay());

}

void InchJoint::initParameters()
{
  k_sp << 1, 1;

}

void InchJoint::initTimerCallback()
{
  calc_angle_timer_ = nh_.createTimer(ros::Duration(0.01), &InchJoint::calc_angle_timer_callback, this);

}

void InchJoint::dynamixel_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
  theta_meas[0] = msg->position.at(0);
  theta_meas[1] = msg->position.at(1);

}

void InchJoint::encoder_phi_callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  phi_meas[0] = msg->data.at(0) * PI / 180;
  phi_meas[1] = msg->data.at(1) * PI / 180;
  
  tau_phi[0] = -k_sp[0] * phi_meas[0];
  tau_phi[1] = -k_sp[1] * phi_meas[1];
}

void InchJoint::calc_angle_timer_callback(const ros::TimerEvent&)
{
  q_meas = theta_meas + phi_meas;
  
  q_dot_meas[0] = inch_q1_dot_->NumDiff(q_meas[0], 0.01);
  q_dot_meas[1] = inch_q2_dot_->NumDiff(q_meas[1], 0.01);
  
  q_ddot_meas[0] = inch_q2_dot_->NumDiff(q_dot_meas[0], 0.01);
  q_ddot_meas[1] = inch_q2_dot_->NumDiff(q_dot_meas[1], 0.01);

  phi_dot_meas[0] = inch_phi1_dot_->NumDiff(phi_meas[0], 0.01);
  phi_dot_meas[1] = inch_phi2_dot_->NumDiff(phi_meas[1], 0.01);

  tau_MCG = calc_MCGDynamics();
}




Eigen::Matrix2d InchJoint::M_Matrix()
{
  Eigen::Matrix2d M_matrix;



  return M_matrix;
}

Eigen::Matrix2d InchJoint::C_Matrix()
{
  Eigen::Matrix2d C_matrix;



  return C_matrix;
}

Eigen::Vector2d InchJoint::G_Matrix()
{
  Eigen::Vector2d G_matrix;



  return G_matrix;
}

Eigen::Vector2d InchJoint::calc_MCGDynamics()
{
  Eigen::Vector2d tau_MCG;
  Eigen::Matrix2d M;
  Eigen::Matrix2d C;
  Eigen::Vector2d G;
  M = M_Matrix();
  C = C_Matrix();
  G = G_Matrix();

  tau_MCG = M * q_ddot_meas + C * q_dot_meas + G;

  return tau_MCG;
}


void InchJoint::test()
{
  ROS_INFO("Here is Inch Joint Toolbox!");
}

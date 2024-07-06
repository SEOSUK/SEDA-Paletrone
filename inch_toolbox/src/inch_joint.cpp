#include "inch_toolbox/inch_joint.h"

InchJoint::InchJoint()
: nh_(""), priv_nh_("~")
{



  initPublisher();
  initSubscriber();
  initTimerCallback();


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
}

void InchJoint::encoder_phi_callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  phi_meas[0] = msg->data.at(0) * PI / 180;
  tau_phi[0] = -k_sp[0] * phi_meas[0];
}

void InchJoint::calc_angle_timer_callback(const ros::TimerEvent&)
{
  q_meas = theta_meas + phi_meas;
}


void InchJoint::test()
{
  ROS_INFO("Here is Inch Joint Toolbox!");
}

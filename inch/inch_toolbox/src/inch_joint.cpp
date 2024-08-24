#include "inch_toolbox/inch_joint.h"
#include "cmath"

InchJoint::InchJoint()
: nh_(""), priv_nh_("")
{
  Link1_length = priv_nh_.param<double>("Link1_length", 0);
  Link2_length = priv_nh_.param<double>("Link2_length", 0);

  Link1_COM = priv_nh_.param<double>("Link1_COM", 0);
  Link2_COM = priv_nh_.param<double>("Link2_COM", 0);

  Link1_mass = priv_nh_.param<double>("Link1_mass", 0);
  Link2_mass = priv_nh_.param<double>("Link2_mass", 0);

  Link1_spring = priv_nh_.param<double>("Link1_spring", 0);
  Link2_spring = priv_nh_.param<double>("Link2_spring", 0);

  Gravity = priv_nh_.param<double>("Gravity", 0);

  N1 = priv_nh_.param<double>("N1", 0);
  N2 = priv_nh_.param<double>("N2", 0);
  N3 = priv_nh_.param<double>("N3", 0);

phi_offset << 0, 0;

  initPublisher();
  initSubscriber();
  initTimerCallback();
  initParameters();  

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
  dynamixel_callback_sub_ = nh_.subscribe("/inch/joint_states", 10, &InchJoint::dynamixel_callback, this);
  encoder_phi_callback_sub_ = nh_.subscribe("/inch/phi_encoder", 10, &InchJoint::encoder_phi_callback, this);

}

void InchJoint::initParameters()
{
  k_sp << Link1_spring, Link2_spring;

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
  // // Rigid Manipulator
  // phi_meas[0] = msg->data.at(0) * PI / 180;
  // phi_meas[1] = msg->data.at(1) * PI / 180;

  // include SEDA
  phi_meas[0] = msg->data.at(0) * PI / 180 + phi_offset[0];
  phi_meas[1] = msg->data.at(1) * PI / 180 + phi_offset[1];

  // 구한 phi_offset은 phi_meas에 계속해서 반영
      
  tau_phi[0] = -k_sp[0] * phi_meas[0];
  tau_phi[1] = -k_sp[1] * phi_meas[1];

  // ROS_INFO("phi1: [%lf]  phi2: [%lf]", phi_meas[0], phi_meas[1]);
}
void InchJoint::calc_angle_timer_callback(const ros::TimerEvent&)
{
  q_meas = theta_meas + phi_meas;

  q_dot_meas[0] = inch_q1_dot_->NumDiff(q_meas[0], 0.01);
  q_dot_meas[1] = inch_q2_dot_->NumDiff(q_meas[1], 0.01);
  
  q_ddot_meas[0] = inch_q1_ddot_->NumDiff(q_dot_meas[0], 0.01);
  q_ddot_meas[1] = inch_q2_ddot_->NumDiff(q_dot_meas[1], 0.01);

  phi_dot_meas[0] = inch_phi1_dot_->NumDiff(phi_meas[0], 0.01);
  phi_dot_meas[1] = inch_phi2_dot_->NumDiff(phi_meas[1], 0.01);

  tau_MCG = calc_MCGDynamics();
}




Eigen::Matrix2d InchJoint::M_Matrix()
{
  Eigen::Matrix2d M_matrix;

  M_matrix << N1 + N2 + 2*N3*cos(q_meas[1]), N2 + N3*cos(q_meas[1]),
                     N2 + N3*cos(q_meas[1]),                     N2; 

  return M_matrix;
}

Eigen::Vector2d InchJoint::C_Matrix()
{
  Eigen::Vector2d C_matrix;

  C_matrix << -N3*sin(q_meas[1])*(2*q_dot_meas[0]*q_dot_meas[1] + pow(q_dot_meas[1], 2)),
                                                   N3*pow(q_dot_meas[0], 2)*sin(q_meas[1]);

  return C_matrix;
}

Eigen::Vector2d InchJoint::G_Matrix()
{
  Eigen::Vector2d G_matrix;

  G_matrix << Link1_mass * Gravity * Link1_COM * cos(q_meas[0]) + Link2_mass * Gravity * Link1_length * cos(q_meas[0]) + Link2_mass * Gravity * Link2_COM * cos(q_meas[0] + q_meas[1]),
              Link2_mass * Gravity * Link2_COM * cos(q_meas[0] + q_meas[1]); 

  return G_matrix;
}

Eigen::Vector2d InchJoint::calc_MCGDynamics()
{
  Eigen::Vector2d tau_MCG;
  Eigen::Matrix2d M;
  Eigen::Vector2d C;
  Eigen::Vector2d G;
  M = M_Matrix();
  C = C_Matrix();
  G = G_Matrix();

  tau_MCG = G;
  

  return tau_MCG;
}


/*****************************************************************************
** MPC
*****************************************************************************/
Eigen::Vector2d InchJoint::MPC_controller_2Link(Eigen::Vector2d ref_, double time_loop_)
{
  Eigen::Vector2d v_MPC;
  Eigen::Vector2d theta_MPC;
  Eigen::Vector2d theta_MPC_dot;
  Eigen::Matrix2d M;
  Eigen::Vector2d C;
  Eigen::Vector2d G;

  v_MPC = K3 * ref_ - K1 * q_meas  - K2 * q_dot_meas;
  M = M_Matrix();
  C = C_Matrix();
  G = G_Matrix();


  tau_MPC = M * v_MPC + C + G;
  phi_MPC[0] = - tau_MPC[0] / k_sp[0];
  phi_MPC[1] = - tau_MPC[1] / k_sp[1];
  theta_MPC = ref_ - phi_MPC;

  theta_MPC_dot[0] = NumDiff((theta_MPC[0] - theta_MPC_i[0]) / time_loop_, time_loop_);
  theta_MPC_dot[1] = NumDiff((theta_MPC[1] - theta_MPC_i[1]) / time_loop_, time_loop_);


  theta_MPC_i = theta_MPC;
  // return theta_MPC + damping_coef * q_dot_meas - damping_coef * theta_MPC_dot;
  return theta_MPC;
}



void InchJoint::init_MPC_controller_2Link(double w0_Link1, double zeta_Link1, double w0_Link2, double zeta_Link2)
{
  double k1_Link1 = w0_Link1 * w0_Link1;
  double k2_Link1 = 2 * zeta_Link1 * w0_Link1;
  double k3_Link1 = k1_Link1;

  double k1_Link2 = w0_Link2 * w0_Link2;
  double k2_Link2 = 2 * zeta_Link2 * w0_Link2;
  double k3_Link2 = k1_Link2;
  //   double k1_Link1 = 1;
  // double k2_Link1 = 0.05;
  // double k3_Link1 = 2;

  // double k1_Link2 = 1;
  // double k2_Link2 = 0.05;
  // double k3_Link2 = 2;


  K1 << k1_Link1, 0,
            0,   k1_Link2;
  K2 << k2_Link1, 0,
            0,   k2_Link2;
  K3 << k3_Link1, 0,
            0,   k3_Link2;


  damping_coef << 1, 0,
                  0, 1;        

  damping_coef << 1/15/Link1_spring,    0,
                     0, 1/15/Link2_spring;
            
}


void InchJoint::init_Link1_MPC_controller(double w0_, double zeta_)
{


}


void InchJoint::test()
{
  ROS_INFO("Here is Inch Joint Toolbox!");
}

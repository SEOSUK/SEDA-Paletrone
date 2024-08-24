#include "inch_toolbox/inch_workbench.h"

using namespace inch;

InchWorkbench::InchWorkbench()
: nh_(""), priv_nh_("")
{
  Link1_length = priv_nh_.param<double>("Link1_length", 0);
  Link2_length = priv_nh_.param<double>("Link2_length", 0);

  Link1_COM = priv_nh_.param<double>("Link1_COM", 0);
  Link2_COM = priv_nh_.param<double>("Link2_COM", 0);

  Link1_mass = priv_nh_.param<double>("Link1_mass", 0);
  Link2_mass = priv_nh_.param<double>("Link2_mass", 0);

}

InchWorkbench::~InchWorkbench()
{
  ROS_INFO("Bye InchWorkbench!");
  ros::shutdown();
}

void InchWorkbench::test()
{
  ROS_INFO("HI! Here is InchWorkbench!");
}

Eigen::Vector2d InchWorkbench::InverseKinematics_2dof(Eigen::Vector2d EE_cmd_)
{
  Eigen::Vector2d theta_ref;
  
  theta_ref[1] = acos((pow(EE_cmd_[0],2) +  pow(EE_cmd_[1],2) - pow(Link1_length,2) - pow(Link2_length,2)) / (2*Link1_length*Link2_length));
  theta_ref[0] = atan(EE_cmd_[1] / EE_cmd_[0]) - atan((Link2_length * sin(theta_ref[1])/(Link1_length + Link2_length * cos(theta_ref[1]))));


  if (std::isnan(theta_ref[0]) || std::isnan(theta_ref[1]) ||
    (Link1_length + Link2_length) < sqrt(EE_cmd_[0] * EE_cmd_[0] + EE_cmd_[1] * EE_cmd_[1]))
  {
    ROS_ERROR("Outa Of WorkSpace!!!");
    return theta_ref_i;
  }
  else if (abs(theta_ref[1]) > 106 * PI / 180)
  {
    ROS_ERROR("Joint 2 is in Constraint!!: [%lf]", theta_ref[1]);
    return theta_ref_i;
  }
  else
  {
    theta_ref_i = theta_ref;
    return theta_ref;
  }
}






Eigen::Vector2d InchWorkbench::ForwardKinematics_2dof(Eigen::Vector2d q_meas_)
{
  Eigen::Vector2d EE_meas;

    EE_meas[0] = Link1_length*cos(q_meas_[0]) + Link2_length*cos(q_meas_[0] + q_meas_[1]);
    EE_meas[1] = Link1_length*sin(q_meas_[0]) + Link2_length*sin(q_meas_[0] + q_meas_[1]);

  return EE_meas;
}


void InchWorkbench::init_Admittancey(double admit_mass_y, double admit_damper_y, double admit_spring_y)
{
// y Axis
  admit_y_A << - admit_damper_y / admit_mass_y, - admit_spring_y / admit_mass_y,
                                1,                              0;

  admit_y_B << 1 / admit_mass_y, 0;

  admit_y_state << 0, 0;
  admit_y_state_dot << 0, 0;
}


double InchWorkbench::admittanceControly(double ref, double f_ext, double time_loop)
{
  double y_cmd = 0;
  
  admit_y_state_dot = admit_y_A * admit_y_state + admit_y_B * f_ext;
  admit_y_state = admit_y_state + admit_y_state_dot * time_loop;

  admit_y_state[1] = saturation(admit_y_state[1],0.2);
  y_cmd = ref - admit_y_state[1];


  return y_cmd;
}

void InchWorkbench::init_Admittancez(double admit_mass_z, double admit_damper_z, double admit_spring_z)
{
// z Axis
  admit_z_A << - admit_damper_z / admit_mass_z, - admit_spring_z / admit_mass_z,
                                1,                              0;

  admit_z_B << 1 / admit_mass_z, 0;

  admit_z_state << 0, 0;
  admit_z_state_dot << 0, 0;
}



double InchWorkbench::admittanceControlz(double ref, double f_ext, double time_loop)
{
  double z_cmd = 0;
  
  admit_z_state_dot = admit_z_A * admit_z_state + admit_z_B * f_ext;
  admit_z_state = admit_z_state + admit_z_state_dot * time_loop;

  admit_z_state[1] = saturation(admit_z_state[1],0.15);
  z_cmd = ref - admit_z_state[1];
  
  return z_cmd;
}


void InchWorkbench::init_CKAdmittancey(double CKadmit_damper_y_, double CKadmit_spring_y_)
{
// y Axis
  CKadmit_damper_y = CKadmit_damper_y_;
  CKadmit_spring_y = CKadmit_spring_y_;
  CKadmit_y_state = 0;
}

double InchWorkbench::CKadmittanceControly(double ref, double f_ext, double time_loop)
{
  CKadmit_y_state_dot = f_ext / CKadmit_damper_y - CKadmit_spring_y / CKadmit_damper_y * CKadmit_y_state;
  CKadmit_y_state = CKadmit_y_state + CKadmit_y_state_dot * time_loop;


  CKadmit_y_state = saturation(CKadmit_y_state, 0.07);

  return ref - CKadmit_y_state;
}


void InchWorkbench::init_CKAdmittancez(double CKadmit_damper_z_, double CKadmit_spring_z_)
{
// y Axis
  CKadmit_damper_z = CKadmit_damper_z_;
  CKadmit_spring_z = CKadmit_spring_z_;
  CKadmit_z_state = 0;
}

double InchWorkbench::CKadmittanceControlz(double ref, double f_ext, double time_loop)
{
  CKadmit_z_state_dot = f_ext / CKadmit_damper_z - CKadmit_spring_z / CKadmit_damper_z * CKadmit_z_state;
  CKadmit_z_state = CKadmit_z_state + CKadmit_z_state_dot * time_loop;

  CKadmit_z_state = saturation(CKadmit_z_state, 0.13);

  return ref - CKadmit_z_state;
}

Eigen::Vector2d InchWorkbench::ForceEstimation(Eigen::Vector2d q_meas_, Eigen::Vector2d tau_ext_)
{
  Eigen::Vector2d force_ext;
  Eigen::Vector2d tau_extT;
  Eigen::Matrix2d J;
  Eigen::Matrix2d JT;
  Eigen::Matrix2d JTI;  

  J = Jacobian(q_meas_);
  JT = J.transpose();
  JTI = JT.inverse();  
  tau_extT = tau_ext_.transpose();
  force_ext = JTI*tau_extT;

  return force_ext;
}


Eigen::Matrix2d InchWorkbench::Jacobian(Eigen::Vector2d q_meas_)
{
  Eigen::Matrix2d jacobian;

  jacobian << -Link1_length*sin(q_meas_[0]) - Link2_length*sin(q_meas_[0] + q_meas_[1]), -Link2_length*sin(q_meas_[0] + q_meas_[1]),
               Link1_length*cos(q_meas_[0]) + Link2_length*cos(q_meas_[0] + q_meas_[1]),  Link2_length*cos(q_meas_[0] + q_meas_[1]);

  return jacobian;
}

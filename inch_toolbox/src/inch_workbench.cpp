#include "inch_toolbox/inch_workbench.h"

using namespace inch;

InchWorkbench::InchWorkbench()
: nh_(""), priv_nh_("")
{
  length_1 = priv_nh_.param<double>("length_1", 0);
  length_2 = priv_nh_.param<double>("length_2", 0);
  length_3 = priv_nh_.param<double>("length_3", 0);


  ROS_INFO("workbench length [%lf] [%lf] [%lf]", length_1, length_2, length_3);
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
  
  theta_ref[1] = acos((pow(EE_cmd_[0],2) +  pow(EE_cmd_[1],2) - pow(length_1,2) - pow(length_2,2)) / (2*length_1*length_2));
  theta_ref[0] = atan(EE_cmd_[1] / EE_cmd_[0]) - atan((length_2 * sin(theta_ref[1])/(length_1 + length_2 * cos(theta_ref[1]))));

  return theta_ref;
}

Eigen::Vector2d InchWorkbench::ForwardKinematics_2dof(Eigen::Vector2d q_meas_)
{
  Eigen::Vector2d EE_meas;

    EE_meas[0] = length_1*cos(q_meas_[0]) + length_2*cos(q_meas_[0] + q_meas_[1]);
    EE_meas[1] = length_1*sin(q_meas_[0]) + length_2*sin(q_meas_[0] + q_meas_[1]);

  return EE_meas;
}

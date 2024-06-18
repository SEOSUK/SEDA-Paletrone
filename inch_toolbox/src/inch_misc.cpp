#include "inch_toolbox/inch_misc.h"

using namespace inch;

InchMisc::InchMisc()
: nh_(""), priv_nh_("")
{
  bw_2nd_B = bw_2nd_B.transpose();
  bw_2nd_state = bw_2nd_state.transpose();
  bw_2nd_state_dot = bw_2nd_state_dot.transpose();
}

InchMisc::~InchMisc()
{
  ROS_INFO("Bye InchMisc!");
  ros::shutdown();
}

void InchMisc::test()
{
  ROS_INFO("HI! Here is InchMisc!");
}

double InchMisc::butterworth_2nd_filter(double input_data_, double time_loop_)
{
  //why not doing?
  // FUcKing.. fix. later..
  bw_2nd_state_dot = bw_2nd_A * bw_2nd_state + bw_2nd_B * input_data_;
  
  bw_2nd_state = bw_2nd_state + bw_2nd_state_dot * time_loop_;

  double output_data = bw_2nd_state[1];

  return output_data;
}

void InchMisc::init_butterworth_2nd_filter(double cut_off_freq_)
{
  bw_2nd_A << -sqrt(2) * cut_off_freq_, - cut_off_freq_ * cut_off_freq_,
                          1,                            0;
  bw_2nd_B << cut_off_freq_ * cut_off_freq_, 0;

  bw_2nd_C << 0, 1;

  bw_2nd_state << 0, 0;

  bw_2nd_state_dot << 0, 0;

  ROS_INFO("INIT butterworth_2nd_filter");
}

double InchMisc::PID_controller()
{
  ROS_INFO("PID..");
  

  return 0;
}

void InchMisc::init_PID_controller(double Kp, double Ki, double Kd)
{
  ROS_INFO("INITPID..");

}

double InchMisc::Dead_Zone_filter()
{

  return 0;
}

void InchMisc::init_Dead_Zone_filter(double dead_zone_max_, double dead_zone_min_)
{


}
#ifndef INCH_MISC_H_
#define INCH_MISC_H_

#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <cmath>
#define PI 3.141592


namespace inch
{
  class InchMisc
  {
  public:
    InchMisc();
    ~InchMisc();

    /*****************************************************************************
    ** Define variables
    *****************************************************************************/ 


    /*****************************************************************************
    ** Define functions
    *****************************************************************************/ 
    void test();  
    double butterworth_2nd_filter(double input_data_, double time_loop_);
    void init_butterworth_2nd_filter(double cut_off_freq_);
    double PID_controller(double ref_, double meas_, double time_loop_);
    void init_PID_controller(double Kp_, double Ki_, double Kd_, double cut_off_freq_);
    double Dead_Zone_filter(double input_data_);
    void init_Dead_Zone_filter(double dead_zone_max_, double dead_zone_min_);
    double NumDiff(double input_data_, double time_loop_);
    double debugger_saturation(double input_data_);
    double tanh_function(double input_data, double cut_off_force);
    
  private:
    /*****************************************************************************
    ** ROS NodeHandle
    *****************************************************************************/
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

    /*****************************************************************************
    ** Define variables
    *****************************************************************************/
    //for butterworth 2nd filter
    Eigen::Matrix2d bw_2nd_A;
    Eigen::Vector2d bw_2nd_B;
    Eigen::Vector2d bw_2nd_C;
    Eigen::Vector2d bw_2nd_state;
    Eigen::Vector2d bw_2nd_state_dot;

    //for PID controller
    double Kp;
    double Ki;
    double Kd;
    double error_dot;
    double error_sum;
    double error_i;
    double cut_off_freq;
    double input_data_prev;
    double dead_zone_max;
    double dead_zone_min;

    

    /*****************************************************************************
    ** Define functions
    *****************************************************************************/
  };
} // namespace INCH

#endif /*INCH_MISC_H_*/

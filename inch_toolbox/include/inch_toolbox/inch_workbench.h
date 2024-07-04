#ifndef INCH_WORKBENCH_H_
#define INCH_WORKBENCH_H_

#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/JointState.h>


namespace inch
{
  class InchWorkbench
  {
  public:
    InchWorkbench();
    ~InchWorkbench();

    /*****************************************************************************
    ** Define variables
    *****************************************************************************/ 
    double length_1;
    double length_2;
    double length_3;

    double admit_spring_y;
    double admit_damper_y;
    double admit_mass_y;
    Eigen::Matrix2d admit_y_A;
    Eigen::Vector2d admit_y_B;
    Eigen::Vector2d admit_y_C;
    Eigen::Vector2d admit_y_state;
    Eigen::Vector2d admit_y_state_dot;
    

    /*****************************************************************************
    ** Define functions
    *****************************************************************************/ 
    void test();
    void init_Admittance(double m, double d, double k);
    double admittanceControly(double ref, double f_ext, double time_loop);


    Eigen::Vector2d InverseKinematics_2dof(Eigen::Vector2d EE_cmd_);
    Eigen::Vector2d ForwardKinematics_2dof(Eigen::Vector2d q_meas_);
    Eigen::Matrix2d Jacobian(Eigen::Vector2d q_meas_);
    Eigen::Vector2d ForceEstimation(Eigen::Vector2d q_meas_, Eigen::Vector2d tau_ext_);


  private:
    /*****************************************************************************
    ** ROS NodeHandle
    *****************************************************************************/
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

    /*****************************************************************************
    ** Define variables
    *****************************************************************************/


    /*****************************************************************************
    ** Define functions
    *****************************************************************************/
  };
} // namespace INCH

#endif /*INCH_WORKBENCH_H_*/
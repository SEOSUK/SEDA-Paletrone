#ifndef INCH_WORKBENCH_H_
#define INCH_WORKBENCH_H_

#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <inch_toolbox/inch_misc.h>


namespace inch
{
  class InchWorkbench : public inch::InchMisc
  {
  public:
    InchWorkbench();
    ~InchWorkbench();

    /*****************************************************************************
    ** Define variables
    *****************************************************************************/ 
    double Link1_length;
    double Link2_length;

    double Link1_COM;
    double Link2_COM;

    double Link1_mass;
    double Link2_mass;
  

    double admit_spring_y;
    double admit_damper_y;
    double admit_mass_y;
    Eigen::Matrix2d admit_y_A;
    Eigen::Vector2d admit_y_B;
    Eigen::Vector2d admit_y_C;
    Eigen::Vector2d admit_y_state;
    Eigen::Vector2d admit_y_state_dot;

    double admit_spring_z;
    double admit_damper_z;
    double admit_mass_z;
    Eigen::Matrix2d admit_z_A;
    Eigen::Vector2d admit_z_B;
    Eigen::Vector2d admit_z_C;
    Eigen::Vector2d admit_z_state;
    Eigen::Vector2d admit_z_state_dot;


    double CKadmit_damper_y;
    double CKadmit_spring_y;
    double CKadmit_y_state_dot;
    double CKadmit_y_state;

    double CKadmit_damper_z;
    double CKadmit_spring_z;
    double CKadmit_z_state_dot;
    double CKadmit_z_state;



    Eigen::Vector2d theta_ref_i;
    Eigen::Vector2d EE_command_vel_limit;

    /*****************************************************************************
    ** Define functions
    *****************************************************************************/ 
    void test();
    void init_Admittancey(double admit_mass_y, double admit_damper_y, double admit_spring_y);
    void init_Admittancez(double admit_mass_z, double admit_damper_z, double admit_spring_z);
    double admittanceControly(double ref, double f_ext, double time_loop);
    double admittanceControlz(double ref, double f_ext, double time_loop);
    void init_CKAdmittancey(double CKadmit_damper_y_, double CKadmit_spring_y_);
    double CKadmittanceControly(double ref, double f_ext, double time_loop);
    void init_CKAdmittancez(double CKadmit_damper_z_, double CKadmit_spring_z_);
    double CKadmittanceControlz(double ref, double f_ext, double time_loop);


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

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

    /*****************************************************************************
    ** Define functions
    *****************************************************************************/ 
    void test();  
    Eigen::Vector2d InverseKinematics_2dof(Eigen::Vector2d EE_cmd_);
    Eigen::Vector2d ForwardKinematics_2dof(Eigen::Vector2d q_meas_);



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
#ifndef INCH_JOINT_H_
#define INCH_JOINT_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class InchJoint
{
 public:
  InchJoint();
  ~InchJoint();

  /*****************************************************************************
  ** Define functions
  *****************************************************************************/
  void test();

 private:
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  /*****************************************************************************
  ** Define variables
  *****************************************************************************/
};

#endif /*INCH_JOINT_H_*/

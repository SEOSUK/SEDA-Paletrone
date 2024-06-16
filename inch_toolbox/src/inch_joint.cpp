#include "inch_toolbox/inch_joint.h"

InchJoint::InchJoint()
: nh_(""), priv_nh_("~")
{

}

InchJoint::~InchJoint()
{
  ROS_INFO("Bye InchJoint!");
  ros::shutdown();
}

void InchJoint::test()
{
  ROS_INFO("Here is Inch Joint Toolbox!");
}

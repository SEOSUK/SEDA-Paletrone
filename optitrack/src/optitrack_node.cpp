/**
 * @file optitrack_node.cpp
 * @brief ROS wrapper for spewing OptiTrack data
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 20 March 2019
 */

#include <ros/ros.h>

#include "optitrack/optitrack.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "optitrack");
  ros::NodeHandle nh("~");
  acl::optitrack::OptiTrack optitrack(nh);
  optitrack.spin();
  return 0;
}

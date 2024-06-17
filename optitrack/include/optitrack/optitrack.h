/**
 * @file optitrack.h
 * @brief ROS wrapper for spewing OptiTrack data
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 20 March 2019
 */

#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>

#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include "optitrack/optitrack_client.h"

namespace acl {
namespace optitrack {

  class OptiTrack
  {
  public:
    OptiTrack(const ros::NodeHandle nh);
    ~OptiTrack() = default;

    /**
     * @brief      Handle any new data from the OptiTrack server.
     */
    void spin();
    
  private:
    ros::NodeHandle nh_;

    std::string localIP_;     ///< IP addr of local NIC to use.
    std::string serverIP_;    ///< IP addr of server (for commands)
    std::string multicastIP_; ///< Multicast group (for UDP data)
    std::string topicSubname_;///< /<veh>/<subname>: used for remapping
    int commandPort_;         ///< Port used for sending cmds to server
    int dataPort_;            ///< Port used for multicast data from server
    bool pubResiduals_;       ///< Publish rigid body residual (quality metric)

    std::unique_ptr<agile::OptiTrackClient> client_;

    /**
     * @brief      Convert from OptiTrack-Raw to ENU, with body flu.
     *
     * @param[in]  p     position array (x, y, z)
     * @param[in]  q     quaternion array (x, y, z, w)
     *
     * @return     Pose of rigid body (flu) w.r.t optitrack (ENU)
     */
    geometry_msgs::Pose toENUPose(const double* p, const double* q);

    /**
     * @brief      Low pass filter the transfer time (i.e., the time offset
     *             between windows and linux) as a cheap outlier rejection
     *             method. NOTE: This method can only be used for one signal
     *             because it contains persistent state.
     *
     * @param[in]  alpha  0 trusts measurements, 1 trusts model
     * @param[in]  x      the new measurement
     *
     * @return     the filtered measurement
     */
    int64_t nanotimeLPF(double alpha, int64_t x);
  };

}
}

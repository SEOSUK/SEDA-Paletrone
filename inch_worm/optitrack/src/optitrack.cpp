/**
 * @file optitrack.cpp
 * @brief ROS wrapper for spewing OptiTrack data
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 20 March 2019
 */

#include "optitrack/optitrack.h"

namespace acl {
namespace optitrack {

OptiTrack::OptiTrack(const ros::NodeHandle nh)
  : nh_(nh)
{
  nh.getParam("local", localIP_);
  nh.getParam("server", serverIP_);
  nh.getParam("multicast_group", multicastIP_);
  nh.getParam("command_port", commandPort_);
  nh.getParam("data_port", dataPort_);
  nh.getParam("pub_residuals", pubResiduals_);
  nh.param<std::string>("topic_subname", topicSubname_, "world");

  client_ = std::make_unique<agile::OptiTrackClient>(localIP_, serverIP_, multicastIP_, commandPort_, dataPort_);

  ROS_INFO_STREAM("Local address: " << localIP_);
  ROS_INFO_STREAM("Server address: " << serverIP_);
  ROS_INFO_STREAM("Multicast group: " << multicastIP_);
  ROS_INFO_STREAM("Command port: " << commandPort_);
  ROS_INFO_STREAM("Data port: " << dataPort_);

  if (!client_->initConnection()) {
    ROS_ERROR("Could not initiate communication with OptiTrack server!");
    ros::shutdown();
  }
}

// ----------------------------------------------------------------------------

void OptiTrack::spin()
{
  // keep track of the various publishers
  std::map<int, ros::Publisher> pubs_pose_; // pose of each rigid body / model
  std::map<int, ros::Publisher> pubs_err_;  // tracking residual of each model

  // Do some debouncing on the number of lost packets. It is not critical
  // unless many consecutive packets are being dropped.
  constexpr int MAX_LOST_PACKETS = 4;
  int lostPackets = 0;

  while (ros::ok()) {

    // allow mocap client to receive optitrack packets
    if (!client_->spinOnce()) {
      if (++lostPackets >= MAX_LOST_PACKETS) {
        ROS_ERROR_THROTTLE(1, "Did not receive data from OptiTrack server! (Are cameras on?)");
      } else {
        ROS_WARN_THROTTLE(1, "Dropped packets.");
      }
    } else {
      lostPackets = 0;
    }

    //
    // Process OptiTrack "packets" (i.e., rigid bodies)
    //

    auto mocapPackets = client_->getPackets();
    for (const auto& pkt : mocapPackets) {

      // Skip this rigid body if tracking is invalid
      if (!pkt.tracking_valid) continue;

      // Initialize publisher for rigid body if not exist.
      if (pubs_pose_.find(pkt.rigid_body_id) == pubs_pose_.end()) {
        // clean model name for ROS topic---only keep alphanumeric chars
        std::string name = pkt.model_name;
        name.erase(std::remove_if(name.begin(), name.end(),
              [](auto const& c) -> bool { return !std::isalnum(c); }
            ), name.end());

        ROS_WARN_STREAM("Found '" << name << "'");

        std::string topic = "/" + name + "/" + topicSubname_;
        pubs_pose_[pkt.rigid_body_id] = nh_.advertise<geometry_msgs::PoseStamped>(topic, 1);

        if (pubResiduals_) {
          topic = "/" + name + "/optitrack_residual";
          pubs_err_[pkt.rigid_body_id] = nh_.advertise<std_msgs::Float32>(topic, 1);
        }
      }

      // Get saved publisher and last state
      ros::Publisher pubPose = pubs_pose_[pkt.rigid_body_id];
      ros::Publisher pubErr = pubs_err_[pkt.rigid_body_id];

      // estimate the windows to linux transmit time
      int64_t offset = pkt.transmit_timestamp - pkt.receive_timestamp;
      int64_t transmit_time = nanotimeLPF(0.975, offset);

      // use the estimate to calculate the the time of the associated pose
      uint64_t packet_ntime = pkt.mid_exposure_timestamp - transmit_time;

      // Add timestamp
      geometry_msgs::PoseStamped currentState;
      currentState.header.frame_id = "optitrack";
      currentState.header.stamp = ros::Time(packet_ntime/1e9, packet_ntime%(int64_t)1e9);

      // convert from raw optitrack frame to ENU with body flu
      currentState.pose = toENUPose(pkt.pos, pkt.orientation);

      // Loop through markers and convert positions from NUE to ENU
      // @TODO since the state message does not understand marker locations.

      // Publish ROS state.
      pubPose.publish(currentState);

      // publish tracking residual
      if (pubResiduals_) {
        std_msgs::Float32 msg;
        msg.data = pkt.mean_marker_error;
        pubErr.publish(msg);
      }
    }
  }
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

geometry_msgs::Pose OptiTrack::toENUPose(const double* p, const double* q)
{
  // rigid body (dots) raw (Y-Up, "fur") w.r.t OptiTrack-Raw (Y-Up, "NUE")
  tf2::Transform T_ORBR;
  T_ORBR.setOrigin(tf2::Vector3(p[0], p[1], p[2]));
  T_ORBR.setRotation(tf2::Quaternion(q[0], q[1], q[2], q[3]));

  // transformation from fur to flu / NUE to NED (same for local / global)
  tf2::Transform T_BRB; // body (flu) w.r.t body-raw (fur)
  T_BRB.setIdentity();
  tf2::Quaternion quat; quat.setRPY(0, -M_PI/2, -M_PI/2);
  T_BRB.setRotation(quat);
  tf2::Transform T_OOR = T_BRB.inverse(); // optitrack-raw (NUE) w.r.t optitrack (ENU)

  // rigid body (dots) (flu) w.r.t OptiTrack frame (ROS ENU)
  tf2::Transform T_OB = T_OOR * T_ORBR * T_BRB;

  geometry_msgs::Pose pose;
  tf2::toMsg(T_OB, pose);
  return pose;
}

// ----------------------------------------------------------------------------

int64_t OptiTrack::nanotimeLPF(double alpha, int64_t x)
{
  static int64_t xhat = x;

  xhat = alpha*xhat + (1-alpha)*x;

  return xhat;
}

} // ns optitrack
} // ns acl

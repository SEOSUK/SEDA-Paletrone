ROS OptiTrack Client for Motive 2
=================================

## Usage

In most cases, `roslaunch optitrack optitrack.launch` should be sufficient to start publishing `geometry_msgs/PoseStamped` topics for each rigid body registered in Motive. Note that this launch file expects that the Motive software is running on a computer with IP address `192.168.1.12` and that this package is running on a computer (which is connected to the same network Motive is running on) with `IP address 192.168.119`. These and other settings (like multicast group IP and port numbers) can be specified in the launch file.

## Network Setup

OptiTrack broadcasts (using multicast) packets over the network using UDP. This OptiTrack ROS node must have access to this subnet (e.g., `192.168.1.x`) by connecting an available NIC into the switch and setting its static IP address.

Using `netplan`, the file `/etc/netplan/02-raven-static-vicon.yaml` has

```yaml
network:
  version: 2
  ethernets:
    enp4s0:
      dhcp4: false
      addresses:
        - 192.168.0.19/24
      gateway4: 192.168.0.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
    enp0s31f6:
      link-local: [ipv4]
      dhcp4: false
      addresses:
        - 192.168.1.119/24
```

Verify that you are properly connected to the subnet and that OptiTrack data is streaming using Wireshark (by default, data packets are on port 1511).
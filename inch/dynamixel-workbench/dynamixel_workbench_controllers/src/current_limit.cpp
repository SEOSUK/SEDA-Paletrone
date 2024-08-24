/*******************************************************************************
* D.A.S.O.M
*
* Department of Aerial Manipulator System for Object Manipulation
*
*     https://github.com/S-CHOI-S/D.A.S.O.M.git
*
* Mobile Robotics Lab. (MRL)
* 	  @ Seoul National University of Science and Technology
*
*	  https://mrl.seoultech.ac.kr/index.do
*
*******************************************************************************/

/* Authors: Sol Choi (Jennifer) */

#include <ros/ros.h>
#include <stdio.h>

#include "dynamixel_workbench_controllers/reboot/dynamixel_sdk.h"
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

// Control table addresses
#define ADDR_PRO_CURRENT_LIMIT      38  // Modify this address according to your motor model

// Protocol version
#define PROTOCOL_VERSION            2.0

// Default setting
#define BAUDRATE                    1000000
#define DEVICENAME                  "/dev/ttyUSB0"

// Initialize PortHandler and PacketHandler
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;


void setCurrentLimit(uint8_t id, uint16_t limit) 
{
  int dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_PRO_CURRENT_LIMIT, limit, NULL);
  if (dxl_comm_result != COMM_SUCCESS) 
  {
    ROS_ERROR("Failed to set current limit: [DXL_ID] = %d", id);
  }
  else ROS_WARN("Successed to set current limit: [DXL_ID] = %d, [Current Limit] = %d", id, limit);
}

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "current_limit");
    ros::NodeHandle nh;

    // Initialize PortHandler and PacketHandler
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open port
    if (portHandler->openPort()) {
        ROS_INFO("Succeeded to open the port!");
    } else {
        ROS_ERROR("Failed to open the port!");
        return 1;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE)) {
        ROS_INFO("Succeeded to set the baudrate!");
    } else {
        ROS_ERROR("Failed to set the baudrate!");
        return 1;
    }

    // Set new current limit
    // setCurrentLimit(1, 100);
    // setCurrentLimit(2, 450);
    // setCurrentLimit(3, 350);
    // setCurrentLimit(4, 100);
    // setCurrentLimit(5, 100);
    // setCurrentLimit(6, 100);
    setCurrentLimit(1, 100);
    setCurrentLimit(2, 1000);
    setCurrentLimit(3, 1000);
    setCurrentLimit(4, 1000);
    setCurrentLimit(5, 1000);
    setCurrentLimit(6, 1000);

    // Close port
    portHandler->closePort();

    return 0;
}

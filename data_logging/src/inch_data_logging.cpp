#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <cmath>

#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>



std_msgs::Float64MultiArray data_log;
ros::Publisher data_log_publisher;


void publisherSet()
{
	data_log_publisher.publish(data_log);
}



void global_EE_ref_callback(const geometry_msgs::Twist& msg)
{
	data_log.data[0] = msg.linear.y;
	data_log.data[1] = msg.linear.z;
}

void global_EE_cmd_callback(const geometry_msgs::Twist& msg)
{
	data_log.data[2] = msg.linear.y;
	data_log.data[3] = msg.linear.z;
}

void global_EE_meas_callback(const geometry_msgs::Twist& msg)
{
	data_log.data[4] = msg.linear.y;
	data_log.data[5] = msg.linear.z;
}



void EE_ref_callback(const geometry_msgs::Twist& msg)
{
	data_log.data[6] = msg.linear.y;
	data_log.data[7] = msg.linear.z;
}

void EE_cmd_callback(const geometry_msgs::Twist& msg)
{
	data_log.data[8] = msg.linear.y;
	data_log.data[9] = msg.linear.z;
}

void EE_meas_callback(const geometry_msgs::Twist& msg)
{
	data_log.data[10] = msg.linear.y;
	data_log.data[11] = msg.linear.z;
}



void F_ext_callback(const geometry_msgs::Vector3& msg)
{
	data_log.data[12] = msg.y;
	data_log.data[13] = msg.z;
}

void F_ext_raw_callback(const geometry_msgs::Vector3& msg)
{
	data_log.data[14] = msg.y;
	data_log.data[15] = msg.z;
}



void q_ref_callback(const geometry_msgs::Vector3& msg)
{
	data_log.data[16] = msg.x;
	data_log.data[17] = msg.y;
}

void q_meas_callback(const geometry_msgs::Vector3& msg)
{
	data_log.data[18] = msg.x;
	data_log.data[19] = msg.y;
}


void inchBase_callback(const geometry_msgs::Twist& msg)
{
	data_log.data[20] = msg.linear.x;
	data_log.data[21] = msg.linear.y;
	data_log.data[22] = msg.linear.z;
	data_log.data[23] = msg.angular.x;
	data_log.data[24] = msg.angular.y;
	data_log.data[25] = msg.angular.z;
}

void phi_meas_callback(const geometry_msgs::Vector3& msg)
{
	data_log.data[26] = msg.x;
	data_log.data[27] = msg.y;
}

void theta_cmd_callback(const geometry_msgs::Vector3& msg)
{
	data_log.data[28] = msg.x;
	data_log.data[29] = msg.y;
}

void tau_ext_callback(const geometry_msgs::Vector3& msg)
{
	data_log.data[30] = msg.x;
	data_log.data[31] = msg.y;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv,"inch_data_logging_node");
	data_log.data.resize(40);

	ros::NodeHandle nh;

	data_log_publisher=nh.advertise<std_msgs::Float64MultiArray>("/inch/data_log",10);



	//From TF listener
	ros::Subscriber global_EE_ref_sub_ = nh.subscribe("/inch/global_EE_ref", 1, global_EE_ref_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber global_EE_cmd_sub_ = nh.subscribe("/inch/global_EE_cmd", 1, global_EE_cmd_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber global_EE_meas_sub_ = nh.subscribe("/inch/global_EE_meas", 1, global_EE_meas_callback,ros::TransportHints().tcpNoDelay());

	//From inch_controller
	ros::Subscriber EE_ref_sub_ = nh.subscribe("/inch/EE_ref", 1, EE_ref_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber EE_cmd_sub_ = nh.subscribe("/inch/EE_cmd", 1, EE_cmd_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber EE_meas_sub_ = nh.subscribe("/inch/EE_meas", 1, EE_meas_callback,ros::TransportHints().tcpNoDelay());

	//From inch_controller
	ros::Subscriber F_ext_sub_ = nh.subscribe("/inch/F_ext", 1, F_ext_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber F_ext_raw_sub_ = nh.subscribe("/inch/F_ext_raw", 1, F_ext_raw_callback,ros::TransportHints().tcpNoDelay());

	//From inch_Controller
	ros::Subscriber q_ref_sub_ = nh.subscribe("/inch/q_ref", 1, q_ref_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber q_meas_sub_ = nh.subscribe("/inch/q_meas", 1, q_meas_callback,ros::TransportHints().tcpNoDelay());

	ros::Subscriber phi_meas_sub_ = nh.subscribe("/inch/phi_meas", 1, phi_meas_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber theta_cmd_sub_ = nh.subscribe("/inch/theta_cmd", 1, theta_cmd_callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber tau_ext_sub_ = nh.subscribe("/inch/tau_ext", 1, tau_ext_callback,ros::TransportHints().tcpNoDelay());

	//From TF BroadCaster
	ros::Subscriber inchBase_sub_ = nh.subscribe("/inch/inchBase", 1, inchBase_callback,ros::TransportHints().tcpNoDelay());


	ros::Timer timerPulish_log=nh.createTimer(ros::Duration(1.0/100.0), std::bind(publisherSet));
	ros::spin();
	return 0;
}
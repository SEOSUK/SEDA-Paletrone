#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <inch_toolbox/inch_misc.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace inch;


class InchLogging
{
	public:
	InchLogging();
	~InchLogging();


 	InchMisc *inch_FilterY;
 	InchMisc *inch_FilterZ; 	
 	

	std_msgs::Float64MultiArray data_log;

	geometry_msgs::Twist F_ext_msg;
	geometry_msgs::Twist F_ext_raw_msg;

	ros::Publisher F_ext_publisher;
	ros::Publisher F_ext_raw_publisher;
	ros::Subscriber data_log_sub_;
	
	
	Eigen::Vector2d F_ext_raw;
	Eigen::Vector2d F_ext_;
	Eigen::Vector2d F_ext;


	void data_log_callback(const std_msgs::Float64MultiArray &msg);
	void FilterActivation();
	void FilterCallback();

};



InchLogging::InchLogging()
{
	inch_FilterY = new InchMisc();
	inch_FilterZ = new InchMisc(); 

	data_log.data.resize(40);


	ros::NodeHandle nh;

	inch_FilterY->init_butterworth_2nd_filter(40);
	inch_FilterZ->init_butterworth_2nd_filter(40);

	data_log_sub_ = nh.subscribe("/inch/data_log", 1, &InchLogging::data_log_callback, this, ros::TransportHints().tcpNoDelay());
	F_ext_publisher = nh.advertise<geometry_msgs::Twist>("/inch/F_ext_raw",10);
	F_ext_raw_publisher = nh.advertise<geometry_msgs::Twist>("/inch/F_ext",10);
}


InchLogging::~InchLogging()
{

	ros::shutdown();
}



void InchLogging::data_log_callback(const std_msgs::Float64MultiArray &msg)
{
	data_log = msg;
	ROS_INFO("%lf", data_log.data[14]);
}

void InchLogging::FilterCallback()
{
	F_ext_raw[0] =data_log.data[14];
	F_ext_raw[1] =data_log.data[15];

	F_ext_raw_msg.linear.y = F_ext_raw[0];
	F_ext_raw_msg.linear.z = F_ext_raw[1];

	F_ext_raw_publisher.publish(F_ext_raw_msg);

}


void InchLogging::FilterActivation()
{
	F_ext_[0] = inch_FilterY->butterworth_2nd_filter(F_ext_raw[0], 0.01);
	F_ext_[1] = inch_FilterZ->butterworth_2nd_filter(F_ext_raw[1], 0.01);

	F_ext_[0] = inch_FilterY->Dead_Zone_filter(F_ext_raw[0], 1, -1);
	F_ext_[1] = inch_FilterZ->Dead_Zone_filter(F_ext_raw[1], 1, -1);

	F_ext_msg.linear.y = F_ext_[0];
	F_ext_msg.linear.z = F_ext_[1];	

	F_ext_publisher.publish(F_ext_msg);
}














int main(int argc, char **argv)
{
	ros::init(argc, argv,"logging_node");

	InchLogging log;


	ros::Rate loop(100);
	while(ros::ok())
	{
		log.FilterCallback();
		log.FilterActivation();
		ros::spinOnce();
		loop.sleep();
	}

	return 0;
}
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense> 


double command_position_fromGUI = 0;
double command_position_toDynxel = 0;

double position_from_model = 0;  //  MDK 모델에 의한 position
double position_dot_from_model = 0;
double position_ddot_from_model = 0;

double position_reference = 0;   //  위치 입력값
double position_command = 0;     //  최종 위치 

double measured_velocity_i = 0;
double measured_velocity_f = 0;
double measured_angle = 0;
double estimated_torque = 0;

double estimated_ang_vel = 0;

double time_i_callback = 0;
double time_f_callback = 0;
double time_callback = 0;

double time_i_loop = 0;
double time_f_loop = 0;
double time_loop = 0;
double F = 0;


Eigen::Matrix2d A;
Eigen::Vector2d B;
Eigen::Vector2d C;
Eigen::Vector2d D;
Eigen::Vector2d BT; //B transpose
Eigen::Vector2d DT; //B transpose


Eigen::Vector2d X_from_model_matrix;
Eigen::Vector2d X_dot_from_model_matrix;
Eigen::Vector2d X_from_model_matrix_T;
Eigen::Vector2d X_dot_from_model_matrix_T;



double measured_phi_rad = 0;
double torque_from_phi = 0;
double phi_offset = 0;


double i = 0;
double law_data = 0;
double sine_trajectory;
/////////////////////////////////////////
//-----------------Parameters----------//
double phi_spring_constant = 0.001;
double phi_convert2Radian = 2*3.141592/360;
double torque_constant = 2;
double freq_sin = 0.10;
double Amp_sin = 0.6;



//-----MDK Model ------//
double m = 0.02;
double b = 0.08;
double k = 0.28;

void sin_generator()
{
	i++;
	sine_trajectory = Amp_sin * sin(2 * 3.141592 * freq_sin * i / 250);
	position_reference = sine_trajectory;
}


void commandcallback(const geometry_msgs::Twist::ConstPtr &msg)
{
	// command_position_fromGUI = msg->linear.x;
	// position_reference = command_position_fromGUI;
}

void encoder_phi_callback(const std_msgs::Float64::ConstPtr &msg)
{
	measured_phi_rad = phi_convert2Radian* msg->data;
    torque_from_phi = - measured_phi_rad * torque_constant;
}


int main(int argc, char** argv)
{

	ros::init(argc, argv, "Admittance_Test");
	ros::NodeHandle n;
	ros::Subscriber CommandSub = n.subscribe("/inch/EE_cmd_gui", 10, commandcallback);  // Command From rqt
	ros::Subscriber encoder_phi_sub_ = n.subscribe("/angle_deg", 10, encoder_phi_callback); // Effort, Position, Velocity
	ros::Publisher Commandpub = n.advertise<sensor_msgs::JointState>("/goal_dynamixel_position", 100); // Final Angle Command
	ros::Publisher test_Pub_ = n.advertise<geometry_msgs::Twist>("/test", 100); //이것저것 테스트용 퍼블리셔입니다

	sensor_msgs::JointState cmd;
//	ros::Subscriber hapticCallback_sub = n.subscribe("/now_haptic_endEffector_publisher", 10, hapticCallback);
 

//-----State Space Representation----//
	A <<  0, 	1,
		-k/m, -b/m;

	B << 0, 1/m;

	C << 1, 0;

	D << 0, 0;

	BT = B.transpose();

	X_from_model_matrix << 0, 0;
	X_dot_from_model_matrix << 0, 0;

	X_from_model_matrix_T = X_from_model_matrix.transpose();
	X_dot_from_model_matrix_T = X_dot_from_model_matrix.transpose();

	time_i_loop = ros::Time::now().toSec(); // 키자마자 퍽 튀기 방지용

	ros::Rate rate(250);



	while(ros::ok())
	{

	sin_generator();


	sensor_msgs::JointState cmd;
	geometry_msgs::Twist test_topic;

	time_f_loop = ros::Time::now().toSec();
	time_loop = time_f_loop - time_i_loop;
	time_i_loop = ros::Time::now().toSec();
	





	X_dot_from_model_matrix_T = A * X_from_model_matrix_T + BT * torque_from_phi;

	X_from_model_matrix_T = X_from_model_matrix_T + X_dot_from_model_matrix_T * time_loop;

	position_from_model = X_from_model_matrix_T[0];

	position_command = position_reference;// - position_from_model; //- measured_phi_rad;

	//ROS_INFO("%lf, %lf, %lf, %lf", X_from_model_matrix_T[0], X_from_model_matrix_T[1], X_dot_from_model_matrix_T[0], X_dot_from_model_matrix_T[1]);
//	ROS_INFO("position_from_model = %lf, position_reference = %lf, estimated_torque = %lf, time_loop = %lf", position_from_model, position_reference, estimated_torque, time_loop);
	cmd.position.push_back(position_command);
	Commandpub.publish(cmd);

	test_topic.linear.x = position_command;
	test_topic.linear.y = position_reference;
	test_topic.linear.z = measured_phi_rad;
	test_topic.angular.x = sine_trajectory;
	test_Pub_.publish(test_topic);

	ros::spinOnce();
	rate.sleep();

	}

	return 0;


}
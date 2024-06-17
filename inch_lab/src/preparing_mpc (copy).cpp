#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense> 
#include "inch_lab/admittanceSRV.h"
#include "inch_lab/circleSRV.h"
#define PI 3.14159265


double command_position_fromGUI = 0;

double position_from_model_x = 0;  //  MDK 모델에 의한 position
double position_dot_from_model_x = 0;

double position_from_model_y = 0;  //  MDK 모델에 의한 position
double position_dot_from_model_y = 0;


double time_i_loop = 0;
double time_f_loop = 0;
double time_loop = 0;
double F = 0;

//--For Circular trajectory --//
double radius = 0;
double freq = 0;
double reps = 0; //서비스 마저 만들자~
double joystick_command_Flag = true;
//-- end --//


Eigen::Vector2d position_reference;   //  위치 입력값
Eigen::Vector2d position_reference_vel_limit;
Eigen::Vector2d position_command;     //  최종 위치 
Eigen::Vector2d joystick_command;
Eigen::Vector2d End_Effector_Position_meas;
Eigen::Vector2d init_position; // 내가 설정한 초기 위치
Eigen::Vector2d first_position; //그냥 노드 켰을 때 위치



Eigen::Matrix2d A_x;
Eigen::Vector2d B_x;
Eigen::Vector2d C_x;
Eigen::Vector2d D_x;
Eigen::Vector2d BT_x; //B transpose
Eigen::Vector2d DT_x; //B transpose


Eigen::Vector2d X_from_model_matrix_x;
Eigen::Vector2d X_dot_from_model_matrix_x;
Eigen::Vector2d X_from_model_matrix_T_x;
Eigen::Vector2d X_dot_from_model_matrix_T_x;

Eigen::Matrix2d A_y;
Eigen::Vector2d B_y;
Eigen::Vector2d C_y;
Eigen::Vector2d D_y;
Eigen::Vector2d BT_y; //B transpose
Eigen::Vector2d DT_y; //B transpose

Eigen::Vector2d phi_;


Eigen::Vector2d X_from_model_matrix_y;
Eigen::Vector2d X_dot_from_model_matrix_y;
Eigen::Vector2d X_from_model_matrix_T_y;
Eigen::Vector2d X_dot_from_model_matrix_T_y;

Eigen::Vector2d angle_meas;
Eigen::Vector2d angle_cmd;
Eigen::Vector2d angle_safe;
Eigen::Vector2d angle_max;
Eigen::Vector2d angle_min;
Eigen::Vector2d angle_real;
Eigen::Vector2d angle_real_cmd;
Eigen::Vector2d external_force;
Eigen::Vector2d external_torque;

Eigen::Matrix2d J;
Eigen::Matrix2d JT;
Eigen::Matrix2d JTI;
Eigen::Vector2d gravity_mat;

Eigen::Vector2d phi_cmd;
Eigen::Vector2d theta_cmd;
Eigen::Vector2d phi_meas;
Eigen::Vector2d phi_ref;
Eigen::Vector2d angle_cmd_vel_limit;



Eigen::Vector2d torque_from_phi;
Eigen::Vector2d deadzone_max;
Eigen::Vector2d deadzone_min;

double phi_offset = 0;


double i = 0;
double law_data = 0;
double sine_trajectory;
/////////////////////////////////////////
//-----------------Parameters----------//
Eigen::Vector2d torque_constant;
double phi_spring_constant = 0.001;
double phi_convert2Radian = 2*3.141592/360;
double freq_sin = 0.10;
double Amp_sin = 0.6;

double m_x = 0;
double b_x = 0;
double k_x = 0;

double m_y = 0;
double b_y = 0;
double k_y = 0;

double length_1 = 0.20075;
double length_2 = 0.149 + 0.053;
double mass_1 = 0.340;
double mass_2 = 0.14;
double com_1 = 0.14; // 0.117 
double com_2 = 0.045;

double EE_vel_limit = 0.001;  // [m/s]
double angle_vel_limit = 0.001;  // [m/s]



void init_admittance_x();
void init_admittance_y();

void initParameters()
{
	torque_constant << 1, 1;

	init_position << 0.2, 0.15;

	//-----MDK Model ------//
	m_x = 100000;
	b_x = 100000;
	k_x = 100000;

	m_y = 100000;
	b_y = 100000;
	k_y = 100000;

    angle_max << 130 * phi_convert2Radian, 130 * phi_convert2Radian;
    angle_min << -130 * phi_convert2Radian, -130 * phi_convert2Radian;

	deadzone_max << 0, 0;
	deadzone_min << 0, 0;
	
	EE_vel_limit = 0.1;  // [m/s] end_effector velocity_limit
	angle_vel_limit = 0.01;  // [rad/s] angle_vel_limit

	phi_ref << 0, 0;
}


bool admittanceCallback(inch_lab::admittanceSRV::Request  &req,
                                      inch_lab::admittanceSRV::Response &res)
{

  m_x = req.m_x;
  b_x = req.d_x;
  k_x = req.k_x;
  
  m_y = req.m_y;
  b_y = req.d_y;
  k_y = req.k_y;

	init_admittance_x();
	init_admittance_y();

  ROS_WARN("MDK Changed!");

  return true;
}

bool circleCallback(inch_lab::circleSRV::Request  &req,
                                      inch_lab::circleSRV::Response &res)
{
  joystick_command_Flag = false;

  radius = req.radius;
  
  freq = req.freq;
  reps = req.reps;

  ROS_INFO("radius: [%lf], freq: [%lf], reps: [%lf]", radius, freq, reps);
  return true;

}

void sin_generator()
{
	i++;
	sine_trajectory = Amp_sin * sin(2 * 3.141592 * freq_sin * i / 250);
}


void init_admittance_x()
{
	A_x <<  0, 	1,
		-k_x/m_x, -b_x/m_x;

	B_x << 0, 1/m_x;

	C_x << 1, 0;

	D_x << 0, 0;

	BT_x = B_x.transpose();

	X_from_model_matrix_x << 0, 0;
	X_dot_from_model_matrix_x << 0, 0;

	X_from_model_matrix_T_x = X_from_model_matrix_x.transpose();
	X_dot_from_model_matrix_T_x = X_dot_from_model_matrix_x.transpose();
}


void calc_admittance_x()
{
	X_dot_from_model_matrix_T_x = A_x * X_from_model_matrix_T_x + BT_x * external_force[0];

	X_from_model_matrix_T_x = X_from_model_matrix_T_x + X_dot_from_model_matrix_T_x * time_loop;

	position_from_model_x = X_from_model_matrix_T_x[0];

	position_command[0] = position_reference[0] - position_from_model_x; //- phi_meas;
}

void init_admittance_y()
{
	A_y <<  0, 	1,
		-k_y/m_y, -b_y/m_y;

	B_y << 0, 1/m_y;

	C_y << 1, 0;

	D_y << 0, 0;

	BT_y = B_y.transpose();

	X_from_model_matrix_y << 0, 0;
	X_dot_from_model_matrix_y << 0, 0;

	X_from_model_matrix_T_y = X_from_model_matrix_y.transpose();
	X_dot_from_model_matrix_T_y = X_dot_from_model_matrix_y.transpose();
}

void calc_admittance_y()
{
	X_dot_from_model_matrix_T_y = A_y * X_from_model_matrix_T_y + BT_y * external_force[1];

	X_from_model_matrix_T_y = X_from_model_matrix_T_y + X_dot_from_model_matrix_T_y * time_loop;

	position_from_model_y = X_from_model_matrix_T_y[0];

	position_command[1] = position_reference[1] - position_from_model_y; //- phi_meas;
}

Eigen::Matrix2d Jacobian(Eigen::Vector2d measured_angle)
{
	Eigen::Matrix2d J;

	J << -length_1 * sin(measured_angle[0]) - length_2 * sin(measured_angle[0] + measured_angle[1]), -length_2 * cos(measured_angle[0] + measured_angle[1]),
					-length_2 * sin(measured_angle[0] + measured_angle[1]), 						 -length_2 * sin(measured_angle[0] + measured_angle[1]);

	return J;
}

Eigen::Vector2d phi_controller(Eigen::Vector2d phi_ref, Eigen::Vector2d phi_meas)
{
	Eigen::Vector2d phi_error_;
	Eigen::Vector2d phi_cmd_;


	phi_error_ = phi_ref - phi_meas;

	// phi_cmd_ = phi_error_;


	return phi_cmd_;
}



void estimate_external_force()
{
	//calc gravity
	gravity_mat[0] = com_1 * cos(angle_real[0]) * mass_1 * 9.81 + (length_1 * cos(angle_meas[0]) + com_2 * cos(angle_meas[0]+angle_meas[1])) * mass_2 * 9.81;
	gravity_mat[1] = com_2 * cos(angle_real[0] + angle_real[1]) * mass_2 * 9.81;

	external_torque = torque_from_phi - gravity_mat;
  	J = Jacobian(angle_real);
  	JT = J.transpose();
  	JTI = JT.inverse();

	external_force = JTI * external_torque;

	for(int i=0; i++; i<2)
	{
		if (external_force[i] <= deadzone_max[i] && external_force[i] >= deadzone_min[i]) external_force[i] = 0;
		else if (external_force[i] > deadzone_max[i]) external_force[i] -= deadzone_max[i];
		else if (external_force[i] < deadzone_min[i]) external_force[i] -= deadzone_min[i];
	}


	// ROS_INFO("JTI \n %lf, %lf\n %lf, %lf",	JTI(0,0), JTI(0,1), JTI(1,0), JTI(1,1));
	// ROS_INFO("external_torque \n %lf, %lf",	external_torque[0], external_torque[1]);
}

void dynamixel_angle_Callback(const sensor_msgs::JointState &msg)
{
    angle_meas[0] = msg.position.at(0);
    angle_meas[1] = msg.position.at(1);

	angle_real = angle_meas; //옵티트랙 키면 이거 끄자
}

Eigen::Vector2d Forward_Kinematics(Eigen::Vector2d angle_meas)
{
    //측정값으로부터 FK 풀기
    Eigen::Vector2d End_Effector_Position_meas_;
    End_Effector_Position_meas_[0] = length_1*cos(angle_meas[0]) + length_2*cos(angle_meas[0] + angle_meas[1]);
    End_Effector_Position_meas_[1] = length_1*sin(angle_meas[0]) + length_2*sin(angle_meas[0] + angle_meas[1]);

    return End_Effector_Position_meas_;
}

void commandcallback(const geometry_msgs::Twist::ConstPtr &msg)
{
	// command_position_fromGUI = msg->linear.x;
	// position_reference = command_position_fromGUI;
}

// void encoder_phi_callback_1(const std_msgs::Float64::ConstPtr &msg)
// {
// 	phi_meas[0] = phi_convert2Radian* msg->data;
//     torque_from_phi[0] = - phi_meas[0] * torque_constant[0];
// }

// void encoder_phi_callback_2(const std_msgs::Float64::ConstPtr &msg)
// {
// 	phi_meas[1] = phi_convert2Radian* msg->data;
//     torque_from_phi[1] = - phi_meas[1] * torque_constant[1];
// }



Eigen::Vector2d Inverse_Kinematics(Eigen::Vector2d End_Effector_Position_cmd)
{
    //GUI에서 받아온 Cmd로 IK 풀기
    Eigen::Vector2d angle_cmd_;
    angle_cmd_[1] = acos((pow(End_Effector_Position_cmd[0],2) +  pow(End_Effector_Position_cmd[1],2) - pow(length_1,2) - pow(length_2,2)) / (2*length_1*length_2));
    angle_cmd_[0] = atan(End_Effector_Position_cmd[1] / End_Effector_Position_cmd[0]) - atan((length_2 * sin(angle_cmd_[1])/(length_1 + length_2 * cos(angle_cmd_[1]))));

    return angle_cmd_;
}

void joystickCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
	joystick_command[0] = - msg->linear.x;
	joystick_command[1] = msg->linear.y;
	position_reference = joystick_command + init_position;
}

void optitrack_sub_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
	angle_real[0] = msg->linear.z;
	angle_real[1] = msg->angular.z;

	phi_meas = angle_real - angle_meas;

	torque_from_phi[0] = -phi_meas[0] * torque_constant[0];
	torque_from_phi[1] = -phi_meas[1] * torque_constant[1];
}

Eigen::Vector2d Angle_Safe_Function(Eigen::Vector2d angle_cmd)
{
    //로봇팔 부러짐 방지 코드
    if(std::isnan(angle_cmd[0]) || std::isnan(angle_cmd[1])) ROS_WARN("Out of Workspace");
    if(angle_cmd[0] > angle_max[0] || angle_cmd[0] < angle_min[0]) ROS_ERROR("angle 1 is LIMIT");
    if(angle_cmd[1] > angle_max[1] || angle_cmd[1] < angle_min[1]) ROS_ERROR("angle 2 is LIMIT");

    if(std::isnan(angle_cmd[0]) || std::isnan(angle_cmd[1]) ||
       angle_cmd[0] > angle_max[0] || angle_cmd[0] < angle_min[0] || 
       angle_cmd[1] > angle_max[1] || angle_cmd[1] < angle_min[1])
    {
        ROS_FATAL("IK ERROR!!");
    }
    else
    {
        return angle_cmd;
    }
	
}

void trajectory_generator()
{
	if(reps>0)
	{
		reps = reps - time_loop;
		position_reference[0] = radius * cos(2*PI*freq*reps) + 0.15;
		position_reference[1] = radius * sin(2*PI*freq*reps) + 0.2;
	}
	else joystick_command_Flag = true;
}

void EE_cmd_Velocity_Limit()
{
  for (int i = 0; i < 2; i++)
  {
    if((position_reference[i] - position_reference_vel_limit[i]) > 0.001) 
    {
      position_reference_vel_limit[i] = position_reference_vel_limit[i] + EE_vel_limit * time_loop;
    }
    else if ((position_reference[i] - position_reference_vel_limit[i]) < - 0.001) 
    {
      position_reference_vel_limit[i] = position_reference_vel_limit[i] - EE_vel_limit * time_loop;     
    }
    else
    {
      position_reference_vel_limit[i] = position_reference[i];
    }
  }
}

void angle_cmd_Velocity_Limit()
{
  for (int i = 0; i < 2; i++)
  {
    if((angle_cmd[i] - angle_cmd_vel_limit[i]) > 0.001) 
    {
      angle_cmd_vel_limit[i] = angle_cmd_vel_limit[i] + angle_vel_limit * time_loop;
    }
    else if ((angle_cmd[i] - angle_cmd_vel_limit[i]) < - 0.001) 
    {
      angle_cmd_vel_limit[i] = angle_cmd_vel_limit[i] - angle_vel_limit * time_loop;     
    }
    else
    {
      angle_cmd_vel_limit[i] = angle_cmd[i];
    }
  }
}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "Admittance_Test");
	ros::NodeHandle n;
	// ros::Subscriber encoder_phi_sub_ = n.subscribe("/angle_deg", 10, encoder_phi_callback_1); // Effort, Position, Velocity
	// ros::Subscriber encoder_phi_2_sub_ = n.subscribe("/angle_deg1", 10, encoder_phi_callback_2); // Effort, Position, Velocity
	ros::Publisher Commandpub = n.advertise<sensor_msgs::JointState>("/goal_dynamixel_position", 100); // Final Angle Command
    ros::Publisher measured_EE_position_pub_ = n.advertise<geometry_msgs::Twist>("/inch/EE_meas", 10);
	ros::Publisher test_Pub_ = n.advertise<geometry_msgs::Twist>("/test", 100); //이것저것 테스트용 퍼블리셔입니다
  	ros::ServiceServer admittance_Server_ = n.advertiseService("/admittanceSrv", admittanceCallback);
  	ros::ServiceServer Circle_Server_ = n.advertiseService("/circleSrv", circleCallback);


//	ros::Subscriber End_Effector_Position_cmd_sub_ = nh.subscribe("/inch/EE_cmd_gui",100,EE_cmd_gui_Callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber dynamixel_angle_sub_ = n.subscribe("/joint_states", 100, dynamixel_angle_Callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber haptic_sub_ = n.subscribe("/phantom/xyzrpy", 100, joystickCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber optitrack_sub_ = n.subscribe("/inch/Link_angle", 100, optitrack_sub_callback, ros::TransportHints().tcpNoDelay());


	sensor_msgs::JointState cmd;
 
	initParameters();
	init_admittance_x();
	init_admittance_y();


	// --초기값 설정용입니다.
	ros::Rate init_rate(1);
	ros::spinOnce();
	init_rate.sleep();
	ros::spinOnce();
    first_position = Forward_Kinematics(angle_real);
	ROS_INFO("%lf, %lf", init_position[0], init_position[1]);
	// // --초기값 설정용입니다 End


	position_reference_vel_limit = first_position;
	position_reference = init_position;

	time_i_loop = ros::Time::now().toSec(); // 키자마자 퍽 튀기 방지용

	ros::Rate rate(200);


    geometry_msgs::Twist EE_meas;

	while(ros::ok())
	{

	ros::spinOnce();
	ROS_INFO("position_reference_vel_limit \n %lf, %lf",position_reference_vel_limit[0], position_reference_vel_limit[1]);

	sensor_msgs::JointState cmd;
	geometry_msgs::Twist test_topic;

	time_f_loop = ros::Time::now().toSec();
	time_loop = time_f_loop - time_i_loop;
	time_i_loop = ros::Time::now().toSec();

	// estimate_external_force();

	// calc_admittance_x();
	// calc_admittance_y();



	trajectory_generator();  //원 그릴때 사용하는 함수. service 안 키면 작동 안함.
	EE_cmd_Velocity_Limit(); // EE_vel_LIMIT 걸어주는 함수. Position_reference 를 position_reference_vel_limit으로 변환. 
    theta_cmd = Inverse_Kinematics(position_reference_vel_limit);

	phi_cmd = phi_controller(phi_ref, phi_meas);
	angle_cmd = theta_cmd + phi_cmd;

	angle_cmd_Velocity_Limit(); // angle_cmd를 angle_cmd_vel_limit 로 바꿔 줌.
	angle_safe = Angle_Safe_Function(angle_cmd_vel_limit);
    End_Effector_Position_meas = Forward_Kinematics(angle_real);

	// ROS_INFO("FK: [%lf][%lf]", End_Effector_Position_meas[0], End_Effector_Position_meas[1]);
	// ROS_INFO("CMD: [%lf][%lf]", position_reference[0], position_reference[1]);
	// ROS_INFO("===================================");


	cmd.position.push_back(angle_safe[0]);
	cmd.position.push_back(angle_safe[1]);
	Commandpub.publish(cmd);

    EE_meas.linear.x = End_Effector_Position_meas[0];
    EE_meas.linear.y = End_Effector_Position_meas[1];
    measured_EE_position_pub_.publish(EE_meas);


	test_topic.linear.x = position_reference[0];
	test_topic.linear.y = position_reference[1];
	test_topic.angular.x = position_reference_vel_limit[0];
	test_topic.angular.y = position_reference_vel_limit[1];
	test_Pub_.publish(test_topic);





	rate.sleep();

	}

	return 0;


}
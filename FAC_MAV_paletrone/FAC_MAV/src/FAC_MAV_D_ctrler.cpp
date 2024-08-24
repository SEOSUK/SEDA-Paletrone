
 //############################################################### CODE UPDATE HISTORY ###########################################################
//
//		         ███╗   ███╗    ██████╗     ██╗     
//        		 ████╗ ████║    ██╔══██╗    ██║     
//        		 ██╔████╔██║    ██████╔╝    ██║     
//        		 ██║╚██╔╝██║    ██╔══██╗    ██║     
//        		 ██║ ╚═╝ ██║    ██║  ██║    ███████  
//
//2022.05.16 Coaxial-Octorotor version
//2022.06.23 Ground Station Application
//2022.08.XX DOB (Disturbance Observer) Application
//2022.09.05 ESC (Extremum Seeking Control) Application
//2022.09.21 Controller mode selection Application
//2023.08.05 Devide tilt mode rp_rains to r_gains, p_gains(Par,Dar,Iar // Pay,Day,Iay)
//2023.08.05 SA change (4,3) -> (4,4)
//###############################################################################################################################################

#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <std_msgs/String.h>
#include <vector>
#include <cmath>
#include <cstdio>
#include <chrono>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/Imu.h>
//#include "FAC_MAV/FAC_MAV_ctrler.h"

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "FAC_MAV/ArmService.h" //ASDF
#include "FAC_MAV/KillService.h" //ASDF
#include "FAC_MAV/PosCtrlService.h" //ASDF
#include "FAC_MAV/HoverService.h" //ASDF
#include "FAC_MAV/FAC_HoverService.h" //ASDF
//-----For CasADi-----//
#include <casadi/casadi.hpp>
#include "DoubleLinkedList.h"
#include <algorithm>
//--------------------//
#include "nav_msgs/Odometry.h"

double freq=200;//controller loop frequency
double pwm_freq=428.0;//pwm signal frequency 428-> 438 

std::chrono::duration<double> delta_t;
int16_t Sbus[10];
int16_t PWM_d;
int16_t loop_time;
std_msgs::Int16MultiArray PWMs_cmd;
std_msgs::Int32MultiArray PWMs_val;
std_msgs::Float32MultiArray Force;
std_msgs::Float32 mass_topic;
sensor_msgs::JointState rac_servo_value;
sensor_msgs::Imu imu;
geometry_msgs::Quaternion imu_quaternion;
geometry_msgs::Vector3 imu_rpy;
geometry_msgs::Vector3 imu_ang_vel;
geometry_msgs::Vector3 imu_lin_acc;
geometry_msgs::Vector3 angle_d;
geometry_msgs::Vector3 pos;
geometry_msgs::Vector3 t265_lin_vel;
geometry_msgs::Vector3 t265_ang_vel;
geometry_msgs::Vector3 lin_vel;
geometry_msgs::Vector3 prev_lin_vel;
geometry_msgs::Vector3 ang_vel;
geometry_msgs::Quaternion t265_quat;
geometry_msgs::Quaternion rot;
geometry_msgs::Quaternion desired_value;
geometry_msgs::Vector3 desired_pos;
geometry_msgs::Vector3 F_total;
geometry_msgs::Vector3 torque_d;
geometry_msgs::Vector3 force_d;
std_msgs::Float32MultiArray force_cmd;
geometry_msgs::Vector3 desired_lin_vel;
geometry_msgs::Vector3 t265_att;
geometry_msgs::Vector3 filtered_angular_rate;
geometry_msgs::Vector3 lin_acl;
std_msgs::Float32 altitude_d;
std_msgs::Float32 battery_voltage_msg;
std_msgs::Float32 battery_real_voltage;
std_msgs::Float32 dt;
geometry_msgs::Vector3 external_force;
geometry_msgs::Vector3 desired_position_change;
geometry_msgs::Vector3 reference_position;
geometry_msgs::Vector3 force_dhat;
geometry_msgs::Vector3 torque_dhat;
geometry_msgs::Vector3 non_bias_external_force;
geometry_msgs::Vector3 external_torque;
geometry_msgs::Vector3 adaptive_external_force;
geometry_msgs::Vector3 adaptive_external_torque;



bool servo_sw=false;
double theta1_command, theta2_command, theta3_command, theta4_command;
bool start_flag=false;
bool tilting_flag=false;

//Mode selection flag
bool attitude_mode = false;
bool velocity_mode = false;
bool position_mode = false;
bool kill_mode = true;
bool altitude_mode = false;
bool tilt_mode = false;
bool ESC_control = false;
bool admittance_mode = false;
bool DOB_mode = true;
//Thruster_cmd
double F1 = 0;//desired propeller 1 force
double F2 = 0;//desired propeller 2 force
double F3 = 0;//desired propeller 3 force
double F4 = 0;//desired propeller 4 force
double F5 = 0;//desired propeller 5 force
double F6 = 0;//desired propeller 6 force
double F7 = 0;//desired propeller 7 force
double F8 = 0;//desired propeller 8 force

//Global : XYZ  Body : xyz
double e_r_i = 0;//roll error integration
double e_p_i = 0;//pitch error integration
double e_y_i = 0;//yaw error integration
double e_X_i = 0;//X position error integration
double e_Y_i = 0;//Y position error integration
double e_Z_i = 0;//Z position error integration
double e_X_dot_i = 0;//X velocity error integration
double e_Y_dot_i = 0;//Y velocity error integration
double e_Z_dot_i = 0;//Z velocity error integration

double tau_r_d = 0;//roll  desired torque (N.m)
double tau_p_d = 0;//pitch desired torque(N.m)
double tau_y_d = 0;//yaw desired torque (N.m)
double tau_y_d_for_integ = 0;//yaw desired torque (N.m)
double tau_y_d_non_sat=0;//yaw deried torque non-saturation (N.m)

double Thrust_d = 0;//altitude desired thrust(N)
//ud_cmd

double r_d = 0;//desired roll angle
double p_d = 0;//desired pitch angle
double y_d = 0;//desired yaw angle
double y_d_tangent = 0;//yaw increment tangent
double T_d = 0;//desired thrust

//Desired Global position
double X_d = 0;//desired X position
double Y_d = 0;//desired Y position
double Z_d = 0;//desired altitude
double X_d_base = 0;//initial desired X position
double Y_d_base = 0;//initial desired Y position
double Z_d_base = 0;//initial desired Z position

//Global Desired Global velocity
double X_dot_d = 0;
double Y_dot_d = 0;
double Z_dot_d = 0;

//Global desired acceleration
double X_ddot_d = 0;
double Y_ddot_d = 0;
double Z_ddot_d = 0;

double alpha = 0;
double beta = 0;

//Body desired force
double F_xd = 0;
double F_yd = 0;
double F_zd = 0;

//Yaw safety
double yaw_prev = 0;
double yaw_now = 0;
double base_yaw = 0;
int yaw_rotate_count = 0;

//Admittance control value
double X_e = 0;
double Y_e = 0;
double Z_e = 0;
double T_e = 0;
double X_r = 0;
double Y_r = 0;
double Z_r = 0;
double T_y_r = 0;
double X_e_x1=0;
double X_e_x2=0;
double X_e_x1_dot=0;
double X_e_x2_dot=0;
double Y_e_x1=0;
double Y_e_x2=0;
double Y_e_x1_dot=0;
double Y_e_x2_dot=0;
double Z_e_x1=0;
double Z_e_x2=0;
double Z_e_x1_dot=0;
double Z_e_x2_dot=0;

double T_e_x1=0;
double T_e_x2=0;
double T_e_x1_dot=0;
double T_e_x2_dot=0;

double M=0.5;
double D=20.0;
double K=0;
double M_z=0.5;
double D_z=30.0;
double K_z=0;
double M_t=0.5;
double D_t=20.0;
double K_t=0;
double external_force_deadzone=4.0; //N
double external_torque_deadzone=1.0; //N

//Force estimation lpf
double Fe_x_x_dot = 0;
double Fe_y_x_dot = 0;
double Fe_z_x_dot = 0;
double Fe_x_x = 0;
double Fe_y_x = 0;
double Fe_z_x = 0;
double Fe_cutoff_freq = 1.0;
//--------------------------------------------------------

//General dimensions

static double r_arm = 0.3025;// m // diagonal length between thruster x2
static double l_servo = 0.035;
static double mass = 8.0;//changed 24.01.10	!!!!PLEASE CHECK the position_dob_m!!!!
static double r2=sqrt(2);


//Body desired force limit
double F_xd_limit = mass*2.0;
double F_yd_limit = mass*2.0; 


//Body desired force limit

//Propeller constants(DJI E800(3510 motors + 620S ESCs))
static double xi = 0.01;//F_i=k*(omega_i)^2, M_i=b*(omega_i)^2
//--------------------------------------------------------

//General parameters======================================

static double pi = 3.141592;//(rad)
static double g = 9.80665;//(m/s^2)

static double rp_limit = 0.25;//(rad)
static double y_vel_limit = 0.01;//(rad/s)
static double y_d_tangent_deadzone = (double)0.05 * y_vel_limit;//(rad/s)
static double T_limit = 80;//(N) 
static double altitude_limit = 1;//(m)
static double XY_limit = 1.0;
static double XYZ_dot_limit=1;
static double XYZ_ddot_limit=2;
static double alpha_beta_limit=1;
static double hardware_servo_limit=0.3;//0.25->0.3 24.01.11 changed
static double servo_command_limit = 0.3;
static double tau_y_limit = 0.25; // 1.0 -> 1.5 ->3.0 ->1.5 ->1.0 -> 0.5 -> 0.75
static double tau_y_th_limit = 2.25; //2023.08.17 update
double tau_y_th = 0.0; //2023.08.17 update
double tau_y_th_integ = 0.0; //2024.01.16 update

double x_c_hat=0.0;
double y_c_hat=0.0;
double z_c_hat=0.0;
//--------------------------------------------------------

//Control gains===========================================

//integratior(PID) limitation
double integ_limit=30;
double z_integ_limit=100;
double pos_integ_limit=10;
double vel_integ_limit=10;

//Roll,Pitch PID gains
double Pa=3.5;
double Ia=0.4;
double Da=0.5;

//Roll PID gains
double Par=3.5;
double Iar=0.4;
double Dar=0.5;

//Pitch PID gains
double Pap=0.0;
double Iap=0.0;
double Dap=0.0;

//Yaw PID gains
double Py=2.0;
double Dy=0.1;

//Z Velocity PID gains
double Pz=16.0;
double Iz=5.0;
double Dz=15.0;

//XY Velocity PID gains
double Pv=5.0;
double Iv=0.1;
double Dv=5.0;

//Position PID gains
double Pp=3.0;
double Ip=0.1;
double Dp=5.0;

//Conventional Flight Mode Control Gains
double conv_Pa, conv_Ia, conv_Da;
double conv_Py, conv_Dy;
double conv_Pz, conv_Iz, conv_Dz;
double conv_Pv, conv_Iv, conv_Dv;
double conv_Pp, conv_Ip, conv_Dp;

//Tilt Flight Mode Control Gains
double tilt_Par, tilt_Iar, tilt_Dar;
double tilt_Pap, tilt_Iap, tilt_Dap;
double tilt_Py, tilt_Dy;
double tilt_Pz, tilt_Iz, tilt_Dz;
double tilt_Pv, tilt_Iv, tilt_Dv;
double tilt_Pp, tilt_Ip, tilt_Dp;
//--------------------------------------------------------

//Servo angle=============================================
double theta1=0,theta2=0,theta3=0,theta4=0;
//--------------------------------------------------------

//Voltage=================================================
double voltage=22.4;
double voltage_old=22.4;
//--------------------------------------------------------


//-DOB----------------------------------------------------
geometry_msgs::Vector3 dhat;
double fq_cutoff=0.1;//Q filter Cut-off frequency

// Nominal MoI
double J_x = 0.002;
double J_y = 0.002;
double J_z = 0.02;

//Roll DOB
double x_r1=0;
double x_r2=0;
double x_r3=0;
double x_dot_r1=0;
double x_dot_r2=0;
double x_dot_r3=0;

double y_r1=0;
double y_r2=0;
double y_r3=0;
double y_dot_r1=0;
double y_dot_r2=0;
double y_dot_r3=0;

double dhat_r = 0;
double tautilde_r_d=0;

//Pitch DOB
double x_p1=0;
double x_p2=0;
double x_p3=0;
double x_dot_p1=0;
double x_dot_p2=0;
double x_dot_p3=0;

double y_p1=0;
double y_p2=0;
double y_p3=0;
double y_dot_p1=0;
double y_dot_p2=0;
double y_dot_p3=0;

double dhat_p=0;
double tautilde_p_d=0;

//Yaw DOB
double x_y1=0;
double x_y2=0;
double x_y3=0;
double x_dot_y1=0;
double x_dot_y2=0;
double x_dot_y3=0;

double y_y1=0;
double y_y2=0;
double y_y3=0;
double y_dot_y1=0;
double y_dot_y2=0;
double y_dot_y3=0;

double tautilde_y_d=0;
//---External Force Bias Eliminator 
void external_force_bias_eliminator();
int eliminator_iter_num=0;
//int eliminator_integ_num=5000;
int eliminator_iter_max=5000;
double F_ex_bias_integrator=0.0;
double F_ey_bias_integrator=0.0;
double F_ex_bias=0.0;
double F_ey_bias=0.0;
bool measure_external_force_bias=false;
//--------------------------------------------------------
//Function------------------------------------------------
template <class T>
T map(T x, T in_min, T in_max, T out_min, T out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void rpyT_ctrl();
void ud_to_PWMs(double tau_r_des, double tau_p_des, double tau_y_des, double Thrust_des);
double Force_to_PWM(double F);
void jointstateCallback(const sensor_msgs::JointState& msg);
void imu_Callback(const sensor_msgs::Imu& msg);
sensor_msgs::JointState servo_msg_create(double desired_theta1, double desired_theta2, double desired_theta3, double desired_theta4);
void sbusCallback(const std_msgs::Int16MultiArray::ConstPtr& array);
void batteryCallback(const std_msgs::Int16& msg);
void posCallback(const geometry_msgs::Vector3& msg);
void rotCallback(const geometry_msgs::Quaternion& msg);
void filterCallback(const sensor_msgs::Imu& msg);
void t265OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
//void external_force_Callback(const geometry_msgs::Vector3& msg);
//void external_torque_Callback(const geometry_msgs::Vector3& msg);
//void adaptive_external_force_Callback(const geometry_msgs::Vector3& msg);
//void adaptive_external_torque_Callback(const geometry_msgs::Vector3& msg);
void setCM();
void setSA();
void publisherSet();
int32_t pwmMapping(double pwm);
void pwm_Command(double pwm1, double pwm2, double pwm3, double pwm4);
void pwm_Kill();
void pwm_Max();
void pwm_Arm();
void pwm_Calibration();
void pid_Gain_Setting();
void disturbance_Observer();
void sine_wave_vibration();
void get_Rotation_matrix();
void Rotation_matrix();
void external_force_estimation();
void admittance_controller();
void Servo_angle_LPF();
void inch_sineGenerator();

double position_dob_fc=0.1;
double position_dob_m=8.0;

double dhat_X_ddot = 0;
double dhat_Y_ddot = 0; 
double dhat_Z_ddot = 0; 
double X_tilde_r = 0;
double Y_tilde_r = 0;
double Z_tilde_r = 0;
void position_dob();
void force_bandpass_filter();
geometry_msgs::Vector3 filtered_force_dhat;
void force_dob();
double force_dob_fc=1.5;
double force_dob_m = 8.0;
double dhat_F_X = .0;
double dhat_F_Y = .0;
double dhat_F_Z = .0;

double torque_dob_fc=4.7;
double dhat_tau_r = 0;
double dhat_tau_p = 0;
double dhat_tau_y = 0;
void torque_dob();
//Mass update-----------------------------
void mass_update();
double updated_mass=0.0;
double mass_x_dot=0.0;
double mass_x=0.0;
double mass_y=0.0;
double mass_lpf_fc=0.1;
//-------------------------------------------------------

//Publisher Group--------------------------------------
ros::Publisher PWMs;
ros::Publisher goal_dynamixel_position_;
ros::Publisher euler;
ros::Publisher desired_angle;
ros::Publisher Forces;
ros::Publisher desired_torque;
ros::Publisher linear_velocity;
ros::Publisher angular_velocity;
ros::Publisher PWM_generator;
ros::Publisher desired_position;
ros::Publisher position;
ros::Publisher kalman_angular_vel;
ros::Publisher kalman_angular_accel;
ros::Publisher desired_force;
ros::Publisher battery_voltage;
ros::Publisher force_command;
ros::Publisher delta_time;
ros::Publisher desired_velocity;
ros::Publisher Center_of_Mass;
ros::Publisher angular_Acceleration;
ros::Publisher sine_wave_data;
ros::Publisher disturbance;
ros::Publisher linear_acceleration;
//ros::Publisher External_force_data;
ros::Publisher reference_desired_pos_error;
ros::Publisher reference_pos;
ros::Publisher force_dhat_pub;
ros::Publisher torque_dhat_pub;
ros::Publisher filtered_force_dhat_pub;
ros::Publisher mass_pub;
ros::Publisher non_bias_external_force_pub;
ros::Publisher hpf_filtered_force_dhat_pub;
//ros::Publisher tau_yaw_thrust;
//----------------------------------------------------

//Control Matrix---------------------------------------
//Eigen::MatrixXd CM(4,8);
Eigen::MatrixXd CM(4,4); //Thrust Allocation
Eigen::MatrixXd SA(4,4); //Servo Allocation
//Eigen::Vector4d u;
Eigen::VectorXd u(4);
Eigen::VectorXd F_cmd(4);
Eigen::VectorXd sine_theta_command(4);
Eigen::VectorXd control_by_theta(4);
Eigen::MatrixXd invCM(4,4);
Eigen::MatrixXd invSA(4,4);
//-----------------------------------------------------

//Linear_velocity--------------------------------------
Eigen::Vector3d cam_v;
Eigen::Matrix3d R_v;
Eigen::Vector3d v;
//-----------------------------------------------------

//Attitude--------------------------------------
Eigen::Vector3d cam_att;
//-----------------------------------------------------

//Rotation_Matrix--------------------------------------
Eigen::Matrix3d Rotz;
Eigen::Matrix3d Roty;
Eigen::Matrix3d Rotx;
//-----------------------------------------------------
//Rotation_Matrix_for_force_dob--------------------------------------
Eigen::Matrix3d RotZ;
Eigen::Matrix3d RotY;
Eigen::Matrix3d RotX;
//-----------------------------------------------------


//Timer------------------------------------------------
auto end  =std::chrono::high_resolution_clock::now();
auto start=std::chrono::high_resolution_clock::now();
//-----------------------------------------------------

//Extremum Seeking Control-----------------------------
geometry_msgs::Vector3 prev_angular_Vel;
geometry_msgs::Vector3 angular_Accel;
geometry_msgs::Vector3 CoM;
geometry_msgs::Vector3 sine_wave;

double MoI_x_hat = 0.01;
double MoI_y_hat = 0.01;
double G_XY = 0.2;
double G_Z = 0.5;

double bias_x_c = 0;
double bias_y_c = 0;
double bias_z_c = 0;
double x_c_limit = 0.04;
double y_c_limit = 0.04;
double z_c_limit = 0.1;

//Bandpass filter parameter
double Q_factor=0.1;
double pass_freq_x=3.0;
double pass_freq_y=3.0;
double pass_freq_z=3.0;

//Filter X 
double X_x_1=0;
double X_x_2=0;
double X_x_1_dot=0;
double X_x_2_dot=0;
double Y_x=0;

//Filter Y 
double X_y_1=0;
double X_y_2=0;
double X_y_1_dot=0;
double X_y_2_dot=0;
double Y_y=0;

//Filter Z 
double X_z_1=0;
double X_z_2=0;
double X_z_1_dot=0;
double X_z_2_dot=0;
double Y_z=0;

//high pass filter parameter----------------------------
void force_highpass_filter();

double hpf_freq_x=0.1;
double hpf_freq_y=0.1;
double hpf_freq_z=0.1;

//Filter X 
double hpf_X_x=0;
double hpf_X_x_dot=0;
double hpf_Y_x=0;

//Filter Y 
double hpf_X_y=0;
double hpf_X_y_dot=0;
double hpf_Y_y=0;

//Filter Z 
double hpf_X_z=0;
double hpf_X_z_dot=0;
double hpf_Y_z=0;

geometry_msgs::Vector3 hpf_filtered_force_dhat;
//----------------------------------------------------
double vibration1=0;
double vibration2=0;
double time_count=0;
double Amp_XY=0.5;
double Amp_Z=1.0;
//-----------------------------------------------------
//Accelerometer LPF------------------------------------
double x_ax_dot = 0;
double x_ay_dot = 0;
double x_az_dot = 0;
double x_ax = 0;
double x_ay = 0;
double x_az = 0;
double accel_cutoff_freq = 1.0;
//-----------------------------------------------------
//Position DOB-----------------------------------------
//double pos_dob_cutoff_freq=1.0;
double X_tilde_ddot_d=0.0;
double Y_tilde_ddot_d=0.0;
double Z_tilde_ddot_d=0.0;


Eigen::MatrixXd MinvQ_A(3,3);
Eigen::MatrixXd MinvQ_B(3,1);
Eigen::MatrixXd MinvQ_C(1,3);
Eigen::MatrixXd Q_A(3,3);
Eigen::MatrixXd Q_B(3,1);
Eigen::MatrixXd Q_C(1,3);
Eigen::MatrixXd MinvQ_X_x(3,1);
Eigen::MatrixXd MinvQ_X_x_dot(3,1);
Eigen::MatrixXd MinvQ_X_y(1,1);
Eigen::MatrixXd Q_X_x(3,1);
Eigen::MatrixXd Q_X_x_dot(3,1);
Eigen::MatrixXd Q_X_y(1,1);
Eigen::MatrixXd MinvQ_Y_x(3,1);
Eigen::MatrixXd MinvQ_Y_x_dot(3,1);
Eigen::MatrixXd MinvQ_Y_y(1,1);
Eigen::MatrixXd Q_Y_x(3,1);
Eigen::MatrixXd Q_Y_x_dot(3,1);
Eigen::MatrixXd Q_Y_y(1,1);
Eigen::MatrixXd MinvQ_Z_x(3,1);
Eigen::MatrixXd MinvQ_Z_x_dot(3,1);
Eigen::MatrixXd MinvQ_Z_y(1,1);
Eigen::MatrixXd Q_Z_x(3,1);
Eigen::MatrixXd Q_Z_x_dot(3,1);
Eigen::MatrixXd Q_Z_y(1,1);

//--------------force DOB------------------------------

Eigen::MatrixXd MinvQ_F_A(1,1);
Eigen::MatrixXd MinvQ_F_B(1,1);
Eigen::MatrixXd MinvQ_F_C(1,1);
Eigen::MatrixXd Q_F_A(1,1);
Eigen::MatrixXd Q_F_B(1,1);
Eigen::MatrixXd Q_F_C(1,1);

Eigen::MatrixXd MinvQ_F_X_x(1,1);
Eigen::MatrixXd MinvQ_F_X_x_dot(1,1);
Eigen::MatrixXd MinvQ_F_X_y(1,1);
Eigen::MatrixXd Q_F_X_x(1,1);
Eigen::MatrixXd Q_F_X_x_dot(1,1);
Eigen::MatrixXd Q_F_X_y(1,1);
Eigen::MatrixXd MinvQ_F_Y_x(1,1);
Eigen::MatrixXd MinvQ_F_Y_x_dot(1,1);
Eigen::MatrixXd MinvQ_F_Y_y(1,1);
Eigen::MatrixXd Q_F_Y_x(1,1);
Eigen::MatrixXd Q_F_Y_x_dot(1,1);
Eigen::MatrixXd Q_F_Y_y(1,1);
Eigen::MatrixXd MinvQ_F_Z_x(1,1);
Eigen::MatrixXd MinvQ_F_Z_x_dot(1,1);
Eigen::MatrixXd MinvQ_F_Z_y(1,1);
Eigen::MatrixXd Q_F_Z_x(1,1);
Eigen::MatrixXd Q_F_Z_x_dot(1,1);
Eigen::MatrixXd Q_F_Z_y(1,1);
//-----------------------------------------------------
//--------------torque DOB------------------------------

Eigen::MatrixXd MinvQ_T_A(2,2);
Eigen::MatrixXd MinvQ_T_B(2,1);
Eigen::MatrixXd MinvQ_T_C_x(1,2);
Eigen::MatrixXd MinvQ_T_C_y(1,2);
Eigen::MatrixXd MinvQ_T_C_z(1,2);
Eigen::MatrixXd Q_T_A(2,2);
Eigen::MatrixXd Q_T_B(2,1);
Eigen::MatrixXd Q_T_C(1,2);

Eigen::MatrixXd MinvQ_T_X_x(2,1);
Eigen::MatrixXd MinvQ_T_X_x_dot(2,1);
Eigen::MatrixXd MinvQ_T_X_y(1,1);
Eigen::MatrixXd Q_T_X_x(2,1);
Eigen::MatrixXd Q_T_X_x_dot(2,1);
Eigen::MatrixXd Q_T_X_y(1,1);
Eigen::MatrixXd MinvQ_T_Y_x(2,1);
Eigen::MatrixXd MinvQ_T_Y_x_dot(2,1);
Eigen::MatrixXd MinvQ_T_Y_y(1,1);
Eigen::MatrixXd Q_T_Y_x(2,1);
Eigen::MatrixXd Q_T_Y_x_dot(2,1);
Eigen::MatrixXd Q_T_Y_y(1,1);
Eigen::MatrixXd MinvQ_T_Z_x(2,1);
Eigen::MatrixXd MinvQ_T_Z_x_dot(2,1);
Eigen::MatrixXd MinvQ_T_Z_y(1,1);
Eigen::MatrixXd Q_T_Z_x(2,1);
Eigen::MatrixXd Q_T_Z_x_dot(2,1);
Eigen::MatrixXd Q_T_Z_y(1,1);
//-----------------------------------------------------
//External force with handle---------------------------
double F_ez_handle=0.0;
double handle_r=0.554; // (m)
std_msgs::Float32 external_force_z_handle;
ros::Publisher external_force_z_handle_pub;
//-----------------------------------------------------
//CoM signal processing--------------------------------
double CoM_fc=1.0;
double x_c_x_dot=0.0;
double x_c_x=0.0;
double x_c_y=0.0;
double y_c_x_dot=0.0;
double y_c_x=0.0;
double y_c_y=0.0;
//-----------------------------------------------------
//Servo Angle LPF
double x_th1_dot = 0;
double x_th2_dot = 0;
double x_th3_dot = 0;
double x_th4_dot = 0;

double x_th1 = 0;
double x_th2 = 0;
double x_th3 = 0;
double x_th4 = 0;
double th_cut_off_freq = 10.0;

Eigen::VectorXd servo_LPF(4);
Eigen::VectorXd servo_command(4);
double servo_command1=0.0;
double servo_command2=0.0;
double servo_command3=0.0;
double servo_command4=0.0;

double inch_sine;
double inch_Amp = 0.05;
double inch_period = 5;
double inch_sineFlag = false;
double cnt = 0;

//-----------------------------------------------------

int main(int argc, char **argv){
	
    	ros::init(argc, argv,"t3_mav_controller");

    	std::string deviceName;
    	ros::NodeHandle params("~");
    	params.param<std::string>("device", deviceName, "/gx5");

    	ros::NodeHandle nh;

	//Loading gains from the "t3_mav_controller.launch" file
		//integratior(PID) limitation
		integ_limit=nh.param<double>("attitude_integ_limit",10);
		z_integ_limit=nh.param<double>("altitude_integ_limit",100);
		pos_integ_limit=nh.param<double>("position_integ_limit",10);

		//Center of Mass
		x_c_hat=nh.param<double>("x_center_of_mass",0.0);
		y_c_hat=nh.param<double>("y_center_of_mass",0.0);
		z_c_hat=nh.param<double>("z_center_of_mass",0.0);
		CoM.x = x_c_hat;
		CoM.y = y_c_hat;
		CoM.z = z_c_hat;

		//Conventional Flight Mode Control Gains
			//Roll, Pitch PID gains
			conv_Pa=nh.param<double>("conv_attitude_rp_P_gain",3.5);
			conv_Ia=nh.param<double>("conv_attitude_rp_I_gain",0.4);
			conv_Da=nh.param<double>("conv_attitude_rp_D_gain",0.5);

			//Yaw PID gains
			conv_Py=nh.param<double>("conv_attitude_y_P_gain",2.0);
			conv_Dy=nh.param<double>("conv_attitude_y_D_gain",0.1);

			//Altitude PID gains
			conv_Pz=nh.param<double>("conv_altitude_P_gain",16.0);
			conv_Iz=nh.param<double>("conv_altitude_I_gain",5.0);
			conv_Dz=nh.param<double>("conv_altitude_D_gain",15.0);

			//Velocity PID gains
			conv_Pv=nh.param<double>("conv_velocity_P_gain",5.0);
			conv_Iv=nh.param<double>("conv_velocity_I_gain",1.0);
			conv_Dv=nh.param<double>("conv_velocity_D_gain",5.0);

			//Position PID gains
			conv_Pp=nh.param<double>("conv_position_P_gain",3.0);
			conv_Ip=nh.param<double>("conv_position_I_gain",0.1);
			conv_Dp=nh.param<double>("conv_position_D_gain",5.0);

		//Tilt Flight Mode Control Gains
			//Roll, Pitch PID gains
			tilt_Par=nh.param<double>("tilt_attitude_r_P_gain",3.5);
			tilt_Iar=nh.param<double>("tilt_attitude_r_I_gain",3.5);
			tilt_Dar=nh.param<double>("tilt_attitude_r_D_gain",3.5);
		
			//tilt_Iap=nh.param<double>("tilt_attitude_p_I_gain",0.4);
			//tilt_Dap=nh.param<double>("tilt_attitude_p_D_gain",0.5);

			//Yaw PID gains
			tilt_Py=nh.param<double>("tilt_attitude_y_P_gain",5.0);
			tilt_Dy=nh.param<double>("tilt_attitude_y_D_gain",0.3);

			//Altitude PID gains
			tilt_Pz=nh.param<double>("tilt_altitude_P_gain",15.0);
			tilt_Iz=nh.param<double>("tilt_altitude_I_gain",5.0);
			tilt_Dz=nh.param<double>("tilt_altitude_D_gain",10.0);

			//Velocity PID gains
			tilt_Pv=nh.param<double>("tilt_velocity_P_gain",5.0);
			tilt_Iv=nh.param<double>("tilt_velocity_I_gain",0.1);
			tilt_Dv=nh.param<double>("tilt_velocity_D_gain",5.0);

			//Position PID gains
			tilt_Pp=nh.param<double>("tilt_position_P_gain",3.0);
			tilt_Ip=nh.param<double>("tilt_position_I_gain",0.1);
			tilt_Dp=nh.param<double>("tilt_position_D_gain",5.0);

	//------------------position DoB Q filter------------------
	
	MinvQ_A << -2*position_dob_fc, -2*pow(position_dob_fc,2), -pow(position_dob_fc,3),
                                  1.0,                       0.0,                     0.0,
                                  0.0,                       1.0,                     0.0;

	MinvQ_B << 1.0, 0.0, 0.0;
	MinvQ_C << position_dob_m*pow(position_dob_fc,3), 0.0, 0.0;

	Q_A << -2*position_dob_fc, -2*pow(position_dob_fc,2), -pow(position_dob_fc,3),
                              1.0,                       0.0,                     0.0,
                              0.0,                       1.0,                     0.0;
	Q_B << 1.0, 0.0, 0.0;
	Q_C << 0.0, 0.0, pow(position_dob_fc,3);

	//------------------Force DoB Q filter----------------------
	Q_F_A << -force_dob_fc;

	Q_F_B << 1.0;

	Q_F_C << force_dob_fc;

	MinvQ_F_A << -force_dob_fc;

	MinvQ_F_B << 1.0;

	MinvQ_F_C << force_dob_m*force_dob_fc;

	//------------------Torque DoB Q filter----------------------
	Q_T_A << -r2*torque_dob_fc, -pow(torque_dob_fc,2),
			       1.0,		      0.0;

	Q_T_B << 1.0, 0.0;

	Q_T_C << 0.0,  pow(torque_dob_fc,2);

	MinvQ_T_A <<  -r2*torque_dob_fc, -pow(torque_dob_fc,2),
				    1.0,		   0.0;

	MinvQ_T_B << 1.0, 0.0;

	MinvQ_T_C_x << J_x*pow(torque_dob_fc,2), 				0.0;
	MinvQ_T_C_y << J_y*pow(torque_dob_fc,2), 				0.0;
	MinvQ_T_C_z << J_z*pow(torque_dob_fc,2), 				0.0;

	//Set Control Matrix----------------------------------------
		setCM();
                F_cmd << 0, 0, 0, 0;
	//----------------------------------------------------------
		mass_topic.data=mass;
	//----------------------------------------------------------
		
		
    PWMs = nh.advertise<std_msgs::Int16MultiArray>("PWMs", 1); // PWM 1,2,3,4
    PWM_generator = nh.advertise<std_msgs::Int32MultiArray>("command",1);  // publish to pca9685
    goal_dynamixel_position_  = nh.advertise<sensor_msgs::JointState>("goal_dynamixel_position",100); // desired theta1,2
	euler = nh.advertise<geometry_msgs::Vector3>("angle",1); // roll, pitch, yaw
	desired_angle = nh.advertise<geometry_msgs::Vector3>("desired_angle",100);
	Forces = nh.advertise<std_msgs::Float32MultiArray>("Forces",100); // F 1,2,3,4
	desired_torque = nh.advertise<geometry_msgs::Vector3>("torque_d",100);
	linear_velocity = nh.advertise<geometry_msgs::Vector3>("lin_vel",100);
	angular_velocity = nh.advertise<geometry_msgs::Vector3>("angular_velocity",100);
	desired_position = nh.advertise<geometry_msgs::Vector3>("pos_d",100);
	position = nh.advertise<geometry_msgs::Vector3>("pos",100);
	desired_force = nh.advertise<geometry_msgs::Vector3>("force_d",100);
	battery_voltage = nh.advertise<std_msgs::Float32>("battery_voltage",100);
	force_command = nh.advertise<std_msgs::Float32MultiArray>("force_cmd",100);
	delta_time = nh.advertise<std_msgs::Float32>("delta_t",100);
	desired_velocity = nh.advertise<geometry_msgs::Vector3>("lin_vel_d",100);
	Center_of_Mass = nh.advertise<geometry_msgs::Vector3>("Center_of_Mass",100);
	angular_Acceleration = nh.advertise<geometry_msgs::Vector3>("ang_accel",100);
	sine_wave_data = nh.advertise<geometry_msgs::Vector3>("sine_wave",100);
	disturbance = nh.advertise<geometry_msgs::Vector3>("att_dhat",100);
	linear_acceleration = nh.advertise<geometry_msgs::Vector3>("imu_lin_acl",100);
	///External_force_data = nh.advertise<geometry_msgs::Vector3>("external_force",1);
	reference_desired_pos_error = nh.advertise<geometry_msgs::Vector3>("pos_e",100);
	reference_pos = nh.advertise<geometry_msgs::Vector3>("pos_r",100);
	mass_pub = nh.advertise<std_msgs::Float32>("mass",1);
	non_bias_external_force_pub = nh.advertise<geometry_msgs::Vector3>("non_bias_external_force",1);
	force_dhat_pub = nh.advertise<geometry_msgs::Vector3>("force_dhat",1);	
	torque_dhat_pub = nh.advertise<geometry_msgs::Vector3>("torque_dhat",1);	
	filtered_force_dhat_pub = nh.advertise<geometry_msgs::Vector3>("filtered_force_dhat",1);	
	hpf_filtered_force_dhat_pub = nh.advertise<geometry_msgs::Vector3>("hpf_filtered_force_dhat",1);
	external_force_z_handle_pub = nh.advertise<std_msgs::Float32>("external_force_z_handle",1);	
	//tau_yaw_thrust = nh.advertise<geometry_msgs::Vector3>("pos_r",100);

    	ros::Subscriber dynamixel_state = nh.subscribe("joint_states",100,jointstateCallback,ros::TransportHints().tcpNoDelay());
   	ros::Subscriber att = nh.subscribe("/imu/data",1,imu_Callback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber rc_in = nh.subscribe("/sbus",100,sbusCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber battery_checker = nh.subscribe("/battery",100,batteryCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber t265_pos=nh.subscribe("/t265_pos",100,posCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber t265_rot=nh.subscribe("/t265_rot",100,rotCallback,ros::TransportHints().tcpNoDelay());
	ros::Subscriber t265_odom=nh.subscribe("/rs_t265/odom/sample",100,t265OdomCallback,ros::TransportHints().tcpNoDelay());
	//ros::Subscriber external_force_sub=nh.subscribe("/external_force",1,external_force_Callback,ros::TransportHints().tcpNoDelay());
	//ros::Subscriber external_torque_sub=nh.subscribe("/external_torque",1,external_torque_Callback,ros::TransportHints().tcpNoDelay());
	//ros::Subscriber adaptive_external_force_sub=nh.subscribe("/adaptive_external_force",1,adaptive_external_force_Callback,ros::TransportHints().tcpNoDelay());
	//ros::Subscriber adaptive_external_torque_sub=nh.subscribe("/adaptvie_external_torque",1,adaptive_external_torque_Callback,ros::TransportHints().tcpNoDelay());	
	
	
	
	ros::Timer timerPublish = nh.createTimer(ros::Duration(1.0/200.0),std::bind(publisherSet));
    ros::spin();
    return 0;
}

void publisherSet(){
	
	end=std::chrono::high_resolution_clock::now();
	delta_t=end-start; 
	dt.data=delta_t.count();
	start=std::chrono::high_resolution_clock::now();
	// F << Eigen::MatrixXd::Identity(3,3), delta_t.count()*Eigen::MatrixXd::Identity(3,3),
	// 	     Eigen::MatrixXd::Zero(3,3),                 Eigen::MatrixXd::Identity(3,3);
	//sine_wave_vibration();
	setCM();
	angular_Accel.x = (imu_ang_vel.x-prev_angular_Vel.x)/delta_t.count();
	angular_Accel.y = (imu_ang_vel.y-prev_angular_Vel.y)/delta_t.count();
	angular_Accel.z = (imu_ang_vel.z-prev_angular_Vel.z)/delta_t.count();

	if(!position_mode){
		X_d_base=pos.x;
		Y_d_base=pos.y;
		X_d = X_d_base;
		Y_d = Y_d_base;
		X_r = X_d;
		Y_r = Y_d;
		e_X_i=0;
		e_Y_i=0;
		if(attitude_mode){
			e_X_dot_i=0;
			e_Y_dot_i=0;	
		}	
	}

	if(kill_mode){	
		y_d=imu_rpy.z;	//[J]This line ensures that yaw desired right after disabling the kill switch becomes current yaw attitude
		Z_d_base=pos.z;
		e_r_i = 0;
		e_p_i = 0;
		e_Z_i = 0;
		e_Z_dot_i=0;
		e_X_i=0;
		e_X_dot_i=0;
		e_Y_i=0;
		e_Y_dot_i=0;
		start_flag=false;
		theta1_command=0.0;
		theta2_command=0.0;
		theta3_command=0.0;
		theta4_command=0.0;
		pwm_Kill();
		//ROS_INFO("Kill mode");	
	}
	else{
		//pwm_Command(Sbus[2],Sbus[2],Sbus[2],Sbus[2],Sbus[2],Sbus[2],Sbus[2],Sbus[2]);
		//pwm_Command(1000,1000,1000,Sbus[2],1000,1000,1000,1000);

		rpyT_ctrl();
		//pwm_Arm();		
	
		//ROS_INFO("Arm mode");	
	}

	//pwm_Calibration();	
	angle_d.x=r_d;
	angle_d.y=p_d;
	angle_d.z=y_d;
	desired_pos.x = X_d;
	desired_pos.y = Y_d;
	desired_pos.z = Z_d;
	Force.data.resize(4);
	Force.data[0] = F1;
	Force.data[1] = F2;
	Force.data[2] = F3;
	Force.data[3] = F4;
//	CoM.x = x_c_hat;
//	CoM.y = y_c_hat;
//	CoM.z = z_c_hat;
	PWMs.publish(PWMs_cmd);// PWMs_d value
	euler.publish(imu_rpy);//rpy_act value
	desired_angle.publish(angle_d);//rpy_d value
	Forces.publish(Force);// force conclusion
	goal_dynamixel_position_.publish(servo_msg_create(servo_command1,servo_command2, servo_command3, servo_command4)); // desired theta
	desired_torque.publish(torque_d); // torque desired
	linear_velocity.publish(lin_vel); // actual linear velocity 
	PWM_generator.publish(PWMs_val);
	desired_position.publish(desired_pos);//desired position 
	position.publish(pos); // actual position
	desired_force.publish(force_d); // desired force it need only tilt mode 
	battery_voltage.publish(battery_voltage_msg);
	force_command.publish(force_cmd); //not use
	delta_time.publish(dt);
	desired_velocity.publish(desired_lin_vel);
	Center_of_Mass.publish(CoM);
	angular_velocity.publish(imu_ang_vel);
	angular_Acceleration.publish(angular_Accel);
	sine_wave_data.publish(sine_wave); //not use
	disturbance.publish(dhat);
	linear_acceleration.publish(imu_lin_acc);
	//External_force_data.publish(external_force);
	reference_desired_pos_error.publish(desired_position_change);
	reference_pos.publish(reference_position);
	prev_angular_Vel = imu_ang_vel;
	prev_lin_vel = lin_vel;
	mass_pub.publish(mass_topic);
	non_bias_external_force_pub.publish(non_bias_external_force);
	force_dhat_pub.publish(force_dhat);
	torque_dhat_pub.publish(torque_dhat);
	filtered_force_dhat_pub.publish(filtered_force_dhat);
	hpf_filtered_force_dhat_pub.publish(hpf_filtered_force_dhat);
	external_force_z_handle_pub.publish(external_force_z_handle);

}

void setCM(){
	//Co-rotating type //2023_07_27 update
	CM << (y_c_hat+r_arm/r2)*cos(theta1)+(-(l_servo-z_c_hat)+xi)*sin(theta1)/r2,  (y_c_hat+r_arm/r2)*cos(theta2)+((l_servo-z_c_hat)-xi)*sin(theta2)/r2,   (y_c_hat-r_arm/r2)*cos(theta3)+((l_servo-z_c_hat)-xi)*sin(theta3)/r2,  (y_c_hat-r_arm/r2)*cos(theta4)+(-(l_servo-z_c_hat)+xi)*sin(theta4)/r2,
	      -(x_c_hat-r_arm/r2)*cos(theta1)+((l_servo-z_c_hat)+xi)*sin(theta1)/r2, -(x_c_hat+r_arm/r2)*cos(theta2)+((l_servo-z_c_hat)+xi)*sin(theta2)/r2, -(x_c_hat+r_arm/r2)*cos(theta3)+(-(l_servo-z_c_hat)-xi)*sin(theta3)/r2, -(x_c_hat-r_arm/r2)*cos(theta4)+(-(l_servo-z_c_hat)-xi)*sin(theta4)/r2,
	      -xi*cos(theta1)+(-(x_c_hat-y_c_hat)/r2)*sin(theta1) 			   ,   xi*cos(theta2)+((x_c_hat+y_c_hat)/r2)*sin(theta2)			  ,   -xi*cos(theta3)+((x_c_hat-y_c_hat)/r2)*sin(theta3)			  ,   xi*cos(theta4)-((x_c_hat+y_c_hat)/r2)*sin(theta4),
																   -cos(theta1),                                                          -cos(theta2),                                                           -cos(theta3),                                                           -cos(theta4);
    invCM = CM.inverse();
//	      -xi*cos(theta1)+(y_c_hat-x_c_hat+r2*r_arm)*sin(theta1)/r2,  xi*cos(theta2)+(x_c_hat+y_c_hat+r2*r_arm)*sin(theta2)/r2,  -xi*cos(theta3)+(x_c_hat-y_c_hat+r2*r_arm)*sin(theta3)/r2,  xi*cos(theta4)+(-x_c_hat-y_c_hat+r2*r_arm)*sin(theta4)/r2,
}

void setSA(){
	SA <<  F1/r2,     F2/r2,     -(F3/r2),     -(F4/r2),
	       F1/r2,    -(F2/r2),   -(F3/r2),       F4/r2,
	      r_arm*F1,  r_arm*F2,   r_arm*F3,   r_arm*F4,
	      r_arm*F1, -(r_arm*F2),   r_arm*F3,  -(r_arm*F4);

	/*SA << (r2/(F1*4)),  (r2/(F1*4)), (1/(r_arm*F1*4)),
		(r2/(F2*4)),  (-r2/(F2*4)), (1/(r_arm*F2*4)),
		(-r2/(F3*4)),  (-r2/(F3*4)), (1/(r_arm*F3*4)),
		(-r2/(F4*4)),  (r2/(F4*4)), (1/(r_arm*F4*4));*/		// 2023.08.03 update
	//	  (y_c_hat-x_c_hat+r2*r_arm)*F1/r2, (x_c_hat+y_c_hat+r2*r_arm)*F2/r2, (x_c_hat-y_c_hat+r2*r_arm)*F3/r2, (-x_c_hat-y_c_hat+r2*r_arm)*F4/r2;

//	std::cout << "SA : \n" << SA <<std::endl;
	// pinvSA = SA.completeOrthogonalDecomposition().pseudoInverse();

	invSA = SA.inverse();
}
void rpyT_ctrl() {
	pid_Gain_Setting();
	y_d_tangent=y_vel_limit*(((double)Sbus[0]-(double)1500)/(double)500);
	if(fabs(y_d_tangent)<y_d_tangent_deadzone || fabs(y_d_tangent)>y_vel_limit) y_d_tangent=0;
	y_d+=y_d_tangent;
	if(altitude_mode){
		if(Sbus[2]>1800){
			Z_d-=0.0005;
			//ROS_INFO("UP!!");
		}
		else if(Sbus[2]<1200){
			Z_d+=0.0005;
			//ROS_INFO("DOWN!!");
		}
			
		if(Z_d <-0.7) Z_d=-0.7 + inch_sine;
		if(Z_d > 0) Z_d=0;
	//Z_d = -altitude_limit*(((double)Sbus[2]-(double)1500)/(double)500)-altitude_limit;
	}
	else{
		T_d = -T_limit*(((double)Sbus[2]-(double)1500)/(double)500)-T_limit;
	}
	//ROS_INFO("%lf",T_d);
	
	double e_r = 0;
	double e_p = 0;
	double e_y = 0;
	double e_X = 0;
	double e_Y = 0;
	double e_Z = 0;
	double e_X_dot = 0;
	double e_Y_dot = 0;
		
	//ROS_INFO("%lf",time_count);

	double global_X_ddot = (lin_vel.x - prev_lin_vel.x)/delta_t.count();
	double global_Y_ddot = (lin_vel.y - prev_lin_vel.y)/delta_t.count();
	x_ax_dot=-accel_cutoff_freq*x_ax+global_X_ddot;
	x_ax+=x_ax_dot*delta_t.count();
	lin_acl.x=accel_cutoff_freq*x_ax;
	x_ay_dot=-accel_cutoff_freq*x_ay+global_Y_ddot;
	x_ay+=x_ay_dot*delta_t.count();
	lin_acl.y=accel_cutoff_freq*x_ay;

	e_Z = Z_r - pos.z;
	e_Z_i += e_Z * delta_t.count();	
	if (fabs(e_Z_i) > z_integ_limit) e_Z_i = (e_Z_i / fabs(e_Z_i)) * z_integ_limit;


	if(altitude_mode){
		Z_ddot_d = Pz * e_Z + Iz * e_Z_i - Dz * lin_vel.z;
	}
	else{
		e_Z_i = 0;
		e_Z_dot_i = 0;
			//ROS_INFO("Manual Thrust!!");
	}

	force_dob();
	force_bandpass_filter();
	force_highpass_filter();
	/*if(DOB_mode){

//		disturbance_Observer();
		force_dob();
		torque_dob();
	//	admittance_controller();
	//`	position_dob();
	//	ROS_INFO("DOB mode");	

	}*/

	if (inch_sineFlag) inch_sineGenerator();


	if(position_mode || velocity_mode){
		torque_dob();
		if(position_mode){
			
			X_d = X_d_base - XY_limit*(((double)Sbus[1]-(double)1500)/(double)500);
			Y_d = Y_d_base + XY_limit*(((double)Sbus[3]-(double)1500)/(double)500);
		
			e_X = X_r - pos.x;// X_d-pos.x;
			e_Y = Y_r - pos.y;// Y_d-pos.y; #2023.11.10 update
			e_X_i += e_X * delta_t.count();
			if (fabs(e_X_i) > pos_integ_limit) e_X_i = (e_X_i / fabs(e_X_i)) * pos_integ_limit;
			e_Y_i += e_Y * delta_t.count();
			if (fabs(e_Y_i) > pos_integ_limit) e_Y_i = (e_Y_i / fabs(e_Y_i)) * pos_integ_limit;
	
			X_dot_d = Pp * e_X + Ip * e_X_i - Dp * lin_vel.x;
			Y_dot_d = Pp * e_Y + Ip * e_Y_i - Dp * lin_vel.y;
		}
		if(velocity_mode){
			X_dot_d = -XYZ_dot_limit*(((double)Sbus[1]-(double)1500)/(double)500);
			Y_dot_d = XYZ_dot_limit*(((double)Sbus[3]-(double)1500)/(double)500);
		}	
		if(fabs(X_dot_d) > XYZ_dot_limit) X_dot_d = (X_dot_d/fabs(X_dot_d))*XYZ_dot_limit;
		if(fabs(Y_dot_d) > XYZ_dot_limit) Y_dot_d = (Y_dot_d/fabs(Y_dot_d))*XYZ_dot_limit;
		
		desired_lin_vel.x = X_dot_d;
		desired_lin_vel.y = Y_dot_d;
	
		e_X_dot = X_dot_d - lin_vel.x;
		e_Y_dot = Y_dot_d - lin_vel.y;
		e_X_dot_i += e_X_dot * delta_t.count();
		if (fabs(e_X_dot_i) > vel_integ_limit) e_X_dot_i = (e_X_dot_i / fabs(e_X_dot_i)) * vel_integ_limit;
		e_Y_dot_i += e_Y_dot * delta_t.count();
		if (fabs(e_Y_dot_i) > vel_integ_limit) e_Y_dot_i = (e_Y_dot_i / fabs(e_Y_dot_i)) * vel_integ_limit;

		X_ddot_d = Pv * e_X_dot + Iv * e_X_dot_i - Dv * lin_acl.x;
		Y_ddot_d = Pv * e_Y_dot + Iv * e_Y_dot_i - Dv * lin_acl.y;
		if(fabs(X_ddot_d) > XYZ_ddot_limit) X_ddot_d = (X_ddot_d/fabs(X_ddot_d))*XYZ_ddot_limit;
		if(fabs(Y_ddot_d) > XYZ_ddot_limit) Y_ddot_d = (Y_ddot_d/fabs(Y_ddot_d))*XYZ_ddot_limit;
		
		if(tilt_mode){
			
			X_tilde_ddot_d=X_ddot_d;//-(dhat_F_X/force_dob_m);
			Y_tilde_ddot_d=Y_ddot_d;//-(dhat_F_Y/force_dob_m);
			Z_tilde_ddot_d=Z_ddot_d;//-(dhat_F_Z/force_dob_m);//changed 24.01.11 19:25
			r_d = 0.0;
			p_d = 0.0;
			F_xd = mass*(X_tilde_ddot_d*cos(imu_rpy.z)*cos(imu_rpy.y)+Y_tilde_ddot_d*sin(imu_rpy.z)*cos(imu_rpy.y)-(Z_tilde_ddot_d)*sin(imu_rpy.y));
			F_yd = mass*(-X_tilde_ddot_d*(cos(imu_rpy.x)*sin(imu_rpy.z)-cos(imu_rpy.z)*sin(imu_rpy.x)*sin(imu_rpy.y))+Y_tilde_ddot_d*(cos(imu_rpy.x)*cos(imu_rpy.z)+sin(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z))+(Z_tilde_ddot_d)*cos(imu_rpy.y)*sin(imu_rpy.x));

		if(fabs(F_xd) > F_xd_limit) F_xd = (F_xd/fabs(F_xd))*F_xd_limit;
		if(fabs(F_yd) > F_yd_limit) F_yd = (F_yd/fabs(F_yd))*F_yd_limit;
			//if(position_mode) ROS_INFO("Position & Tilt !!!");
			//else ROS_INFO("Velocity & Tilt !!!");
		}
		else{
			alpha=(-sin(imu_rpy.z)*X_ddot_d+cos(imu_rpy.z)*Y_ddot_d)/g;
			beta=(-cos(imu_rpy.z)*X_ddot_d-sin(imu_rpy.z)*Y_ddot_d)/g;
			if(fabs(alpha) > alpha_beta_limit) alpha = (alpha/fabs(alpha))*alpha_beta_limit;
			if(fabs(beta) > alpha_beta_limit) beta = (beta/fabs(beta))*alpha_beta_limit;
			r_d = asin(alpha);
			p_d = asin(beta/cos(imu_rpy.x));
			if(fabs(r_d)>rp_limit) r_d = (r_d/fabs(r_d))*rp_limit;
			if(fabs(p_d)>rp_limit) p_d = (p_d/fabs(p_d))*rp_limit;
			
			//if(position_mode) ROS_INFO("Position & Conventional !!!");
			//else ROS_INFO("Velocity & Conventional !!!");
		}
	}
	if(attitude_mode){
		if(tilt_mode){
			r_d = 0.0;
			p_d = 0.0;
			F_xd=-mass*XYZ_ddot_limit*(((double)Sbus[1]-(double)1500)/(double)500);
			F_yd=mass*XYZ_ddot_limit*(((double)Sbus[3]-(double)1500)/(double)500);
			//ROS_INFO("Attitude & Tilt !!!");

		}
		else{
			r_d=rp_limit*(((double)Sbus[3]-(double)1500)/(double)500);
			p_d=rp_limit*(((double)Sbus[1]-(double)1500)/(double)500);
			//ROS_INFO("Attidue Control!!");
			F_xd=0.0;
			F_yd=0.0;
			//ROS_INFO("Attitude & Conventional");
		}
	}
	
	e_r = r_d - imu_rpy.x;
	e_p = p_d - imu_rpy.y;
	//e_y = y_d - imu_rpy.z;
	e_y = T_y_r - imu_rpy.z;

	
	e_r_i += e_r * delta_t.count();
	if (fabs(e_r_i) > integ_limit)	e_r_i = (e_r_i / fabs(e_r_i)) * integ_limit;
	e_p_i += e_p * delta_t.count();
	if (fabs(e_p_i) > integ_limit)	e_p_i = (e_p_i / fabs(e_p_i)) * integ_limit;

	tau_r_d = Par * e_r + Iar * e_r_i + Dar * (-imu_ang_vel.x);//- (double)0.48; //Pa -> Par
	tau_p_d = Par * e_p + Iar * e_p_i + Dar * (-imu_ang_vel.y);//+ (double)0.18; //Pa -> Par 
	tau_y_d = Py * e_y + Dy * (-imu_ang_vel.z);
	//tau_y_d_non_sat=tau_y_d;
	//if(fabs(tau_y_d) > tau_y_limit) tau_y_d = tau_y_d/fabs(tau_y_d)*tau_y_limit;
	
	if(altitude_mode){
		desired_lin_vel.z = Z_ddot_d; // But this is desired acceleration
		if(!attitude_mode) F_zd = mass*(X_tilde_ddot_d*(sin(imu_rpy.x)*sin(imu_rpy.z)+cos(imu_rpy.x)*cos(imu_rpy.z)*sin(imu_rpy.y))-Y_tilde_ddot_d*(cos(imu_rpy.z)*sin(imu_rpy.x)-cos(imu_rpy.x)*sin(imu_rpy.y)*sin(imu_rpy.z))+(Z_tilde_ddot_d)*cos(imu_rpy.x)*cos(imu_rpy.y));
		else F_zd = mass*(Z_tilde_ddot_d);
		//ROS_INFO("Altitude");
	}
	else{
		F_zd=T_d;
		//ROS_INFO("Manual Thrust!!");
	}


	if(F_zd >= -0.5*mass*g) F_zd = -0.5*mass*g;
	if(F_zd <= -2.0*mass*g) F_zd = -2.0*mass*g; 
	

	//DOB-----------------------------------------------------
		//disturbance_Observer();
	//--------------------------------------------------------
	//torque_dob();
	tautilde_r_d = tau_r_d- dhat_tau_r;//dhat_r;// - dhat_tau_r; 
	tautilde_p_d = tau_p_d- dhat_tau_p;//dhat_p;// - dhat_tau_p;
	tautilde_y_d = tau_y_d;//- dhat_tau_y;
	
	//u << tau_r_d, tau_p_d, tau_y_d, F_zd;
//	u << tautilde_r_d, tautilde_p_d, tautilde_y_d, F_zd;
	torque_d.x = tau_r_d;
	torque_d.y = tau_p_d;
	torque_d.z = tau_y_d;
	if(admittance_mode)
	{
		admittance_controller();
	//	F_xd = F_xd - dhat_F_X;
//		F_yd = F_yd - dhat_F_Y;
//		F_zd = F_zd - filtered_force_dhat.z;
	//tau_y_d = tau_y_d - external_torque.z;
		x_c_x=0;
		y_c_x=0;
		x_c_hat=CoM.x;
		y_c_hat=CoM.y;
	}
	else{
		x_c_x_dot=-CoM_fc*x_c_x+dhat_tau_p/F_zd;
		x_c_x+=x_c_x_dot*delta_t.count();
		x_c_y=CoM_fc*x_c_x;

		y_c_x_dot=-CoM_fc*y_c_x-dhat_tau_r/F_zd;
		y_c_x+=y_c_x_dot*delta_t.count();
		y_c_y=CoM_fc*y_c_x;

		CoM.x=x_c_hat+x_c_y;
		CoM.y=y_c_hat+y_c_y;
	}
	
	X_r=X_d-X_e;
	Y_r=Y_d-Y_e;
	Z_r=Z_d-Z_e;
	T_y_r = y_d - T_e;

	force_d.x = F_xd - dhat_F_X;
	force_d.y = F_yd- dhat_F_Y;
	force_d.z = F_zd - filtered_force_dhat.z;
	
//	u << tau_r_d, tau_p_d, tau_y_d, F_zd;
	//u << tau_r_d, tau_p_d, tau_y_d-tau_y_sin, F_zd;
	//std::cout << "tau_y_sin : " << tau_y_sin << "    ";	
	prev_angular_Vel = imu_ang_vel;
	//ud_to_PWMs(tau_r_d, tau_p_d, tau_y_d, Thrust_d); //but not use 22.10.12
	ud_to_PWMs(tautilde_r_d, tautilde_p_d, tautilde_y_d, Thrust_d);
}

double asine_safety(double command)
{
	 double sine_theta_limit = 0.9; //rad | rad2deg :: 85.94
	 if(fabs(command)>sine_theta_limit)  command= (command/fabs(command))*sine_theta_limit;
	 return command;
}

double Forces_safety(double command)
{

 /*if(command > 80) command = 80;
 if(command <= 1) command = 1;
*/ 

 return command;
}

void Servo_angle_LPF()
{
  x_th1_dot=-th_cut_off_freq*x_th1+theta1_command;
	x_th1+=x_th1_dot*delta_t.count();
	//ROS_INFO("%lf ",x_th1_dot);
	//ROS_WARN("%lf " ,x_th1);

  x_th2_dot=-th_cut_off_freq*x_th2+theta2_command;
	x_th2+=x_th2_dot*delta_t.count();

  x_th3_dot=-th_cut_off_freq*x_th3+theta3_command;
	x_th3+=x_th3_dot*delta_t.count();

  x_th4_dot=-th_cut_off_freq*x_th4+theta4_command;
	x_th4+=x_th4_dot*delta_t.count();

  servo_LPF(0)=th_cut_off_freq*x_th1;
  servo_LPF(1)=th_cut_off_freq*x_th2;
  servo_LPF(2)=th_cut_off_freq*x_th3;
  servo_LPF(3)=th_cut_off_freq*x_th4;
}

void ud_to_PWMs(double tau_r_des, double tau_p_des, double tau_y_des, double Thrust_des) {	
 	
	//Co-rotating coaxial

	//Conventional type

	tau_y_d_non_sat=tau_y_d;
	if(fabs(tau_y_d_non_sat) > tau_y_limit)
	{
	       	tau_y_d_for_integ = tau_y_d_non_sat/fabs(tau_y_d_non_sat)*tau_y_limit;
		tau_y_d = tau_y_d_for_integ;
	}
	u << tautilde_r_d, tautilde_p_d, tau_y_d, F_zd;

	F_cmd = invCM*u;
	F1 = Forces_safety(F_cmd(0));
	F2 = Forces_safety(F_cmd(1));
	F3 = Forces_safety(F_cmd(2));
	F4 = Forces_safety(F_cmd(3));

	//tau_yaw_sine_desired part---//
	if((fabs(tau_y_d_for_integ)-tau_y_limit)==0)
	{
		tau_y_th = tau_y_d_non_sat-tau_y_d_for_integ;
		if(fabs(tau_y_th) > tau_y_th_limit) tau_y_th = (tau_y_th/fabs(tau_y_th))*tau_y_th_limit;//2023.08.17 update
	}
	tau_y_th_integ +=(1*tau_y_d);
	if(fabs(tau_y_th_integ)>tau_y_limit)
	{
		tau_y_th_integ = (tau_y_th_integ/fabs(tau_y_th_integ))*tau_y_limit;
	}

	/*else{
		tau_y_th=0.0;
	}*/


//	ROS_INFO("%lf ",tau_y_th);
	//----------------------------//
	//	(r_arm*(sin(theta1)*F1 + sin(theta2)*F2 + sin(theta3)*F3 + sin(theta4)*F4)); //2023.08.03 update

	//double tau_y_cos=-F1*xi*cos(theta1)+F2*xi*cos(theta2)-F3*xi*cos(theta3)+F4*xi*cos(theta4);
	control_by_theta << F_xd, F_yd, tau_y_th + tau_y_th_integ, 0;
	//std::cout << "tau_y_cos : " << tau_y_cos << std::endl;	
	//std::cout << "pinvSA : \n" << pinvSA <<std::endl;
	setSA();
	sine_theta_command = invSA*control_by_theta; //2023.08.05 update
	//ROS_INFO("%lf %lf %lf %lf",sine_theta_command(0),sine_theta_command(1),sine_theta_command(2),sine_theta_command(3));
	if(!tilt_mode){
		theta1_command = 0.0;
        	theta2_command = 0.0;
		theta3_command = 0.0;
		theta4_command = 0.0;
	}
	//Tilting type
	else {
		//Servo_angle_LPF();
		theta1_command = asin(asine_safety(sine_theta_command(0)));
		theta2_command = asin(asine_safety(sine_theta_command(1)));
  		theta3_command = asin(asine_safety(sine_theta_command(2)));
  		theta4_command = asin(asine_safety(sine_theta_command(3)));

		Servo_angle_LPF();
  		servo_command1=servo_LPF(0);
  		servo_command2=servo_LPF(1);
  		servo_command3=servo_LPF(2);
		servo_command4=servo_LPF(3);
		/*servo_command1=theta1_command;
		servo_command2=theta2_command;
		servo_command3=theta3_command;
		servo_command4=theta4_command;*/

 		if(fabs(servo_command1)>hardware_servo_limit) servo_command1 = (servo_command1/fabs(servo_command1))*hardware_servo_limit;
 		if(fabs(servo_command2)>hardware_servo_limit) servo_command2 = (servo_command2/fabs(servo_command2))*hardware_servo_limit;
 		if(fabs(servo_command3)>hardware_servo_limit) servo_command3 = (servo_command3/fabs(servo_command3))*hardware_servo_limit;
 		if(fabs(servo_command4)>hardware_servo_limit) servo_command4 = (servo_command4/fabs(servo_command4))*hardware_servo_limit;
		//2023.08.03 udpate
		//ROS_INFO("%lf %lf %lf %lf",theta1_command, theta2_command, theta3_command, theta4_command);
	}
	
	//pwm_Kill();
	pwm_Command(Force_to_PWM(F1),Force_to_PWM(F2), Force_to_PWM(F3), Force_to_PWM(F4));
	//ROS_INFO("1:%d, 2:%d, 3:%d, 4:%d",PWMs_cmd.data[0], PWMs_cmd.data[1], PWMs_cmd.data[2], PWMs_cmd.data[3]);
	// Force_to_PWM(-T_d/4.0);
	// ROS_INFO("%f 1:%d, 2:%d, 3:%d, 4:%d",z_d,PWMs_cmd.data[0], PWMs_cmd.data[1], PWMs_cmd.data[2], PWMs_cmd.data[3]);
}

 

double Force_to_PWM(double F) {
	double pwm;
	double A = -8.1332*pow(10.0,-8.0)*pow(voltage,2.0)+5.5525*pow(10.0,-6.0)*voltage-4.5119*pow(10.0,-5.0);
	double B = 0.00014354*pow(voltage,2.0)-0.0087694*voltage+0.065575;
	double C = -0.028531*pow(voltage,2.0)+1.7194*voltage-6.2575;
	/*
	double param1 = 710;//-B/(2.0*A);
	double param2 = 0.00016;//1.0/A;
	double param3 = 0.00041888;//(pow(B,2.0)-4*A*C)/(4*pow(A,2.0));
	double param4 = 0.00008;
	*/

	double param1 = -B/(2.0*A);
	double param2 = 1.0/A;
	double param3 = (pow(B,2.0)-4*A*C)/(4*pow(A,2.0));
	//	double param4 = 0.00008;
	//Force=A*pwm^2+B*pwm+C
	// A = 0.00004 / B = -0.0568 / C = 17.546 

	if(param2*F+param3>0){
		pwm = param1 + sqrt(param2 * F + param3);
		// ROS_INFO("%lf",pwm);
	}
	else pwm = 1100.;
	if (pwm > 1900)	pwm = 1900;
	if(pwm < 1100) pwm = 1100;
	/*if(altitude_mode){//altitude mode
		if(Z_d_base<=0){
			if(Z_d>Z_d_base && !start_flag) {
				pwm=1100;
			}
			else if(Z_d<Z_d_base) start_flag=true;
		}
		else pwm=param1;
	}*/
	return pwm;
}

void jointstateCallback(const sensor_msgs::JointState& msg){
    	rac_servo_value=msg;
	theta1=msg.position[0];
	theta2=msg.position[1];
	theta3=msg.position[2];
	theta4=msg.position[3];
	
    	//ROS_INFO("theta1:%lf   theta2:%lf  theta3:%lf  theta4:%lf" ,theta1, theta2, theta3, theta4);
}

ros::Time imuTimer;

void imu_Callback(const sensor_msgs::Imu& msg){
    	imu=msg;
    
    	// TP attitude - Quaternion representation
    	imu_quaternion=msg.orientation;
    	imu_ang_vel=msg.angular_velocity;
    	// ROS_INFO("R:%lf\tP:%lf\tY:%lf",imu_ang_vel.x,imu_ang_vel.y,imu_ang_vel.z);
    	imu_lin_acc=msg.linear_acceleration;

    	tf::Quaternion quat;
    	tf::quaternionMsgToTF(imu_quaternion,quat);

    	// TP attitude - Euler representation
    	tf::Matrix3x3(quat).getRPY(imu_rpy.x,imu_rpy.y,imu_rpy.z);
	base_yaw = cam_att(2);
    	if(base_yaw - yaw_prev < -pi) yaw_rotate_count++;
	else if(base_yaw - yaw_prev > pi) yaw_rotate_count--;
	yaw_now = base_yaw+2*pi*yaw_rotate_count;
	//ROS_INFO("now : %lf / prev : %lf / count : %d",yaw_now, yaw_prev, yaw_rotate_count);
	imu_rpy.z = yaw_now;
	yaw_prev = base_yaw;
	// ROS_INFO("imuCallback time : %f",(((double)ros::Time::now().sec-(double)imuTimer.sec)+((double)ros::Time::now().nsec-(double)imuTimer.nsec)/1000000000.));
	//imuTimer = ros::Time::now();
}

void filterCallback(const sensor_msgs::Imu& msg){
	filtered_angular_rate=msg.angular_velocity;
}

sensor_msgs::JointState servo_msg_create(double desired_theta1, double desired_theta2, double desired_theta3, double desired_theta4){
	sensor_msgs::JointState servo_msg;

	servo_msg.header.stamp=ros::Time::now();

	servo_msg.name.resize(4);
	servo_msg.name[0]="id_1";
	servo_msg.name[1]="id_2";
	servo_msg.name[2]="id_3";
	servo_msg.name[3]="id_4";


	servo_msg.position.resize(4);
	servo_msg.position[0]=desired_theta1;
	servo_msg.position[1]=desired_theta2;
	servo_msg.position[2]=desired_theta3;
	servo_msg.position[3]=desired_theta4;
	//ROS_INFO("rr: %lf, rp: %lf",rr,rp);
	return servo_msg;
}

void sbusCallback(const std_msgs::Int16MultiArray::ConstPtr& array){
	for(int i=0;i<10;i++){
		Sbus[i]=map<int16_t>(array->data[i], 352, 1696, 1000, 2000);
	}
	
	if(Sbus[4]<1500) {
		kill_mode=true;
		tilt_mode=false;
	}
	else {
		kill_mode=false;
		tilt_mode=true;
	}
	
	if(Sbus[5]>1500) altitude_mode=true;
	else altitude_mode=false;

	if(Sbus[6]<1300){
		attitude_mode=true;
		velocity_mode=false;
		position_mode=false;
	}
	else if(Sbus[6]<1700){
		attitude_mode=false;
		velocity_mode=true;
		position_mode=false;
	}
	else{
		attitude_mode=false;
		velocity_mode=false;
		position_mode=true;
	}

	/*if(Sbus[7]>1500) DOB_mode=true;
	else DOB_mode=false;*/

	if(Sbus[7]>1500) inch_sineFlag = true;
	else inch_sineFlag = false;
//	ROS_INFO("%d, %d, %d, %d, %d, %d, %d, %d",Sbus[0],Sbus[1],Sbus[2],Sbus[3],Sbus[4],Sbus[5],Sbus[6],Sbus[7]);
	//if(Sbus[9]>1500) ESC_control=true;
	//else ESC_control=false;
}


void batteryCallback(const std_msgs::Int16& msg){
	int16_t value=msg.data;
	voltage=value*5.0/(double)1024/(7440./(30000.+7440.)); //4096
	double kv=0.08;

	voltage=kv*voltage+(1-kv)*voltage_old;
	voltage_old=voltage;
	//ROS_INFO("%f",voltage);
	if(voltage>25.2) voltage=25.2;
	if(voltage<20.0) voltage=20.0;
	battery_voltage_msg.data=voltage;
	
}

ros::Time posTimer;
void posCallback(const geometry_msgs::Vector3& msg){

	pos.x=msg.x;
	pos.y=msg.y;
	pos.z=msg.z;
}

void rotCallback(const geometry_msgs::Quaternion& msg){
	rot.x=msg.x;
	rot.y=msg.y;
	rot.z=msg.z;
	rot.w=msg.w;
	
	tf::Quaternion quat;
	tf::quaternionMsgToTF(rot,quat);
	tf::Matrix3x3(quat).getRPY(t265_att.x,t265_att.y,t265_att.z);
	//ROS_INFO("%f, %f, %f",t265_att.x,t265_att.y,t265_att.z);	
}

void t265OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	t265_lin_vel=msg->twist.twist.linear;
	t265_ang_vel=msg->twist.twist.angular;
	t265_quat=msg->pose.pose.orientation;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(rot,quat);
	tf::Matrix3x3(quat).getRPY(cam_att(0),cam_att(1),cam_att(2));
	cam_v << t265_lin_vel.x, t265_lin_vel.y, t265_lin_vel.z;
	
	
	R_v << 0, -r2/2, -r2/2,
	    	0, r2/2, -r2/2,
		1, 0, 0;
	
	
	v = R_v*cam_v;

	double global_X_dot = v(2)*(sin(imu_rpy.x)*sin(imu_rpy.z)+cos(imu_rpy.x)*cos(imu_rpy.z)*sin(imu_rpy.y))-v(1)*(cos(imu_rpy.x)*sin(imu_rpy.z)-cos(imu_rpy.z)*sin(imu_rpy.x)*sin(imu_rpy.y))+v(0)*cos(imu_rpy.z)*cos(imu_rpy.y);
	double global_Y_dot = v(1)*(cos(imu_rpy.x)*cos(imu_rpy.z)+sin(imu_rpy.x)*sin(imu_rpy.z)*sin(imu_rpy.y))-v(2)*(cos(imu_rpy.z)*sin(imu_rpy.x)-cos(imu_rpy.x)*sin(imu_rpy.z)*sin(imu_rpy.y))+v(0)*cos(imu_rpy.y)*sin(imu_rpy.z);
	double global_Z_dot = -v(0)*sin(imu_rpy.y)+v(2)*cos(imu_rpy.x)*cos(imu_rpy.y)+v(1)*cos(imu_rpy.y)*sin(imu_rpy.x);

	lin_vel.x=global_X_dot;
	lin_vel.y=global_Y_dot;
	lin_vel.z=global_Z_dot;
	//ROS_INFO("Attitude - [r: %f  p: %f  y:%f]",cam_att(0),cam_att(1),cam_att(2));
	//ROS_INFO("Rotate Linear_velocity - [x: %f  y: %f  z:%f]",v(0),v(1),v(2));
	//ROS_INFO("Linear_velocity - [x: %f  y: %f  z:%f]",cam_v(0),cam_v(1),cam_v(2));
	//ROS_INFO("Angular_velocity - [x: %f  y: %f  z:%f]",w(0),w(1),w(2));
}
/*void external_force_Callback(const geometry_msgs::Vector3& msg){
	external_force=msg;

	non_bias_external_force.x=external_force.x-F_ex_bias;
	non_bias_external_force.y=external_force.y-F_ey_bias;
	non_bias_external_force.z=external_force.z;
}

void external_torque_Callback(const geometry_msgs::Vector3& msg){
	external_torque=msg;
}

void adaptive_external_force_Callback(const geometry_msgs::Vector3& msg){
	adaptive_external_force=msg;

//	non_bias_external_force.x=external_force.x-f_ex_bias;
//	non_bias_external_force.y=external_force.y-f_ey_bias;
//	non_bias_external_force.z=external_force.z;
}

void adaptive_external_torque_Callback(const geometry_msgs::Vector3& msg){
	adaptive_external_torque=msg;
}*/

int32_t pwmMapping(double pwm){
	return (int32_t)(65535.*pwm/(1./pwm_freq*1000000.));//65535 ->65635
}

void pwm_Command(double pwm1, double pwm2, double pwm3, double pwm4){
	PWMs_cmd.data.resize(4);
	PWMs_cmd.data[0] = pwm1;
	PWMs_cmd.data[1] = pwm2;
	PWMs_cmd.data[2] = pwm3;
	PWMs_cmd.data[3] = pwm4;
	PWMs_val.data.resize(16);
	PWMs_val.data[0] = pwmMapping(pwm1);
	PWMs_val.data[1] = pwmMapping(pwm2);
	PWMs_val.data[2] = pwmMapping(pwm3);
	PWMs_val.data[3] = pwmMapping(pwm4);
	PWMs_val.data[4] = -1;
	PWMs_val.data[5] = -1;
	PWMs_val.data[6] = -1;
	PWMs_val.data[7] = -1;
	PWMs_val.data[8] = -1;
	PWMs_val.data[9] = -1;
	PWMs_val.data[10] = -1;
	PWMs_val.data[11] = -1;
	PWMs_val.data[12] = -1;
	PWMs_val.data[13] = -1;
	PWMs_val.data[14] = -1;
	PWMs_val.data[15] = -1;

}

void pwm_Max(){
	PWMs_val.data.resize(16);
	PWMs_val.data[0] = pwmMapping(2000.);
	PWMs_val.data[1] = pwmMapping(2000.);
	PWMs_val.data[2] = pwmMapping(2000.);
	PWMs_val.data[3] = pwmMapping(2000.);
	PWMs_val.data[4] = -1;
	PWMs_val.data[5] = -1;
	PWMs_val.data[6] = -1;
	PWMs_val.data[7] = -1;
	PWMs_val.data[8] = -1;
	PWMs_val.data[9] = -1;
	PWMs_val.data[10] = -1;
	PWMs_val.data[11] = -1;
	PWMs_val.data[12] = -1;
	PWMs_val.data[13] = -1;
	PWMs_val.data[14] = -1;
	PWMs_val.data[15] = -1;
}

void pwm_Kill(){
	PWMs_cmd.data.resize(4);
	PWMs_cmd.data[0] = 1000;
	PWMs_cmd.data[1] = 1000;
	PWMs_cmd.data[2] = 1000;
	PWMs_cmd.data[3] = 1000;
	PWMs_val.data.resize(16);
	PWMs_val.data[0] = pwmMapping(1000.);
	PWMs_val.data[1] = pwmMapping(1000.);
	PWMs_val.data[2] = pwmMapping(1000.);
	PWMs_val.data[3] = pwmMapping(1000.);
	PWMs_val.data[4] = -1;
	PWMs_val.data[5] = -1;
	PWMs_val.data[6] = -1;
	PWMs_val.data[7] = -1;
	PWMs_val.data[8] = -1;
	PWMs_val.data[9] = -1;
	PWMs_val.data[10] = -1;
	PWMs_val.data[11] = -1;
	PWMs_val.data[12] = -1;
	PWMs_val.data[13] = -1;
	PWMs_val.data[14] = -1;
	PWMs_val.data[15] = -1;

}

void pwm_Arm(){
	PWMs_cmd.data.resize(4);
	PWMs_cmd.data[0] = 1750;
	PWMs_cmd.data[1] = 1750;
	PWMs_cmd.data[2] = 1750;
	PWMs_cmd.data[3] = 1750;
	PWMs_val.data.resize(16);
	PWMs_val.data[0] = pwmMapping(1500.);
	PWMs_val.data[1] = pwmMapping(1500.);
	PWMs_val.data[2] = pwmMapping(1500.);
	PWMs_val.data[3] = pwmMapping(1750.);
	PWMs_val.data[4] = -1;
	PWMs_val.data[5] = -1;
	PWMs_val.data[6] = -1;
	PWMs_val.data[7] = -1;
	PWMs_val.data[8] = -1;
	PWMs_val.data[9] = -1;
	PWMs_val.data[10] = -1;
	PWMs_val.data[11] = -1;
	PWMs_val.data[12] = -1;
	PWMs_val.data[13] = -1;
	PWMs_val.data[14] = -1;
	PWMs_val.data[15] = -1;

}
void pwm_Calibration(){
	//if(Sbus[4]>1500) pwm_Arm();
	//else pwm_Kill();
        if(kill_mode==true){
		pwm_Kill();
	}
	else{
	       	pwm_Max();
	}
//ROS_INFO("Sbus[6] : %d", Sbus[6]);	
}


void pid_Gain_Setting(){
	if(!tilt_mode){
		Par = conv_Pa;
		Iar = conv_Ia;
		Dar = conv_Da;

		Py = conv_Py;
		Dy = conv_Dy;

		Pz = conv_Pz;
		Iz = conv_Iz;
		Dz = conv_Dz;
		
		Pv = conv_Pv;
		Iv = conv_Iv;
		Dv = conv_Dv;

		Pp = conv_Pp;
		Ip = conv_Ip;
		Dp = conv_Dp;
	}
	else{
		Par = tilt_Par;
		Iar = tilt_Iar;
		Dar = tilt_Dar;


		//Pap = tilt_Pap;
		//Iap = tilt_Iap;
		//Dap = tilt_Dap;


		Py = tilt_Py;
		Dy = tilt_Dy;

		Pz = tilt_Pz;
		Iz = tilt_Iz;
		Dz = tilt_Dz;
		
		Pv = tilt_Pv;
		Iv = tilt_Iv;
		Dv = tilt_Dv;

		Pp = tilt_Pp;
		Ip = tilt_Ip;
		Dp = tilt_Dp;
	}
	//ROS_INFO("%.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf / %.2lf ",Pa, Ia, Da, Py, Dy, Pz, Iz, Dz, Pp, Ip, Dp);
}

void disturbance_Observer(){
	//DOB------------------------------------------------------------------------------
	//Nominal transfer function : q/tau = 1/Js^2    Q - 3rd order butterworth filter
	//Roll
	//Q*(Js^2) transfer function to state space 
	x_dot_r1 = -2*fq_cutoff*x_r1-2*pow(fq_cutoff,2)*x_r2-pow(fq_cutoff,3)*x_r3+imu_rpy.x;
	x_dot_r2 = x_r1;
	x_dot_r3 = x_r2;
    x_r1 += x_dot_r1*delta_t.count(); 
	x_r2 += x_dot_r2*delta_t.count(); 
	x_r3 += x_dot_r3*delta_t.count(); 
	double tauhat_r = J_x*pow(fq_cutoff,3)*x_r1;

	//Q transfer function to state space
	y_dot_r1 = -2*fq_cutoff*y_r1-2*pow(fq_cutoff,2)*y_r2-pow(fq_cutoff,3)*y_r3+tautilde_r_d;//tautilde_r_d
	y_dot_r2 = y_r1;
	y_dot_r3 = y_r2;
	y_r1 += y_dot_r1*delta_t.count();
	y_r2 += y_dot_r2*delta_t.count();
	y_r3 += y_dot_r3*delta_t.count();
	double Qtautilde_r = pow(fq_cutoff,3)*y_r3;

	dhat_r = tauhat_r - Qtautilde_r;


	//Pitch
	//Q*(Js^2) transfer function to state space 
	x_dot_p1 = -2*fq_cutoff*x_p1-2*pow(fq_cutoff,2)*x_p2-pow(fq_cutoff,3)*x_p3+imu_rpy.y;
	x_dot_p2 = x_p1;
	x_dot_p3 = x_p2;
	x_p1 += x_dot_p1*delta_t.count(); 
	x_p2 += x_dot_p2*delta_t.count(); 
	x_p3 += x_dot_p3*delta_t.count(); 
	double tauhat_p = J_y*pow(fq_cutoff,3)*x_p1;

	//Q transfer function to state space
	y_dot_p1 = -2*fq_cutoff*y_p1-2*pow(fq_cutoff,2)*y_p2-pow(fq_cutoff,3)*y_p3+tautilde_p_d;//tautilde_p_d
	y_dot_p2 = y_p1;
	y_dot_p3 = y_p2;
	y_p1 += y_dot_p1*delta_t.count();
	y_p2 += y_dot_p2*delta_t.count();
	y_p3 += y_dot_p3*delta_t.count();
	double Qtautilde_p = pow(fq_cutoff,3)*y_p3;

	dhat_p = tauhat_p - Qtautilde_p;


	//Yaw
	//Q*(Js^2) transfer function to state space 
	x_dot_y1 = -2*fq_cutoff*x_y1-2*pow(fq_cutoff,2)*x_y2-pow(fq_cutoff,3)*x_y3+imu_rpy.z;
	x_dot_y2 = x_y1;
	x_dot_y3 = x_y2;
    x_y1 += x_dot_y1*delta_t.count(); 
	x_y2 += x_dot_y2*delta_t.count(); 
	x_y3 += x_dot_y3*delta_t.count(); 
	double tauhat_y = J_z*pow(fq_cutoff,3)*x_y1;

	//Q transfer function to state space
	y_dot_y1 = -2*fq_cutoff*y_y1-2*pow(fq_cutoff,2)*y_y2-pow(fq_cutoff,3)*y_y3+tautilde_y_d;
	y_dot_y2 = y_y1;
	y_dot_y3 = y_y2;
	y_y1 += y_dot_y1*delta_t.count();
	y_y2 += y_dot_y2*delta_t.count();
	y_y3 += y_dot_y3*delta_t.count();
	double Qtautilde_y = pow(fq_cutoff,3)*y_y3;

	double dhat_y = tauhat_y - Qtautilde_y;
	dhat.x = dhat_r;
	dhat.y = dhat_p;
	dhat.z = dhat_y;

	//tautilde_y_d = tau_y_d - dhat_y;
    tautilde_y_d = tau_y_d;
	//--------------------------------------------------------------------------------------
}
/*
void sine_wave_vibration(){
	vibration1 = Amp_Z*sin(pass_freq1*time_count);
	vibration2 = Amp_XY*sin(pass_freq2*time_count);
	sine_wave.x = vibration1;
	sine_wave.y = vibration2;
	time_count += delta_t.count();
}*/

void get_Rotation_matrix(){
	Rotz << cos(imu_rpy.z), -sin(imu_rpy.z),   0,
	        sin(imu_rpy.z),  cos(imu_rpy.z),   0,
		             0,               0, 1.0;

	Roty << cos(imu_rpy.y),   0, sin(imu_rpy.y),
	                     0, 1.0,              0,
	       -sin(imu_rpy.y),   0, cos(imu_rpy.y);

	Rotx << 1.0,              0,               0,
                  0, cos(imu_rpy.x), -sin(imu_rpy.x),
                  0, sin(imu_rpy.x),  cos(imu_rpy.x);		  
}

void admittance_controller(){
	/*if(fabs(adaptive_external_force.x)<external_force_deadzone) adaptive_external_force.x=0;
	if(fabs(adaptive_external_force.y)<external_force_deadzone) adaptive_external_force.y=0;	
	if(fabs(adaptive_external_force.z)<external_force_deadzone) adaptive_external_force.z=0;*/	
	get_Rotation_matrix();
//	double filtered_dhat_F_x = filtered_force_dhat.x;
//	double filtered_dhat_F_y = filtered_force_dhat.y;
	double dhat_F_z_handle = external_force_z_handle.data;
//	force_bandpass_filter();
	if(fabs(force_dhat.x)<external_force_deadzone) dhat_F_X=0;
	if(fabs(force_dhat.y)<external_force_deadzone) dhat_F_Y=0;	
	if(fabs(external_force_z_handle.data)<5.0) dhat_F_z_handle=0;	// deadzone z direction
	if(fabs(torque_dhat.z)<external_torque_deadzone) dhat_tau_y=0;	
	
	external_force.x = dhat_F_X;
	external_force.y = dhat_F_Y;
	external_force.z = dhat_F_z_handle;
	external_torque.z = dhat_tau_y;

	Eigen::Vector3d Fe;
//	Eigen::Vector3d Te;
	Eigen::Vector3d Fe_for_rotM;
//	Eigen::Vector3d Te_for_rotM;

	Fe_for_rotM << external_force.x, external_force.y, external_force.z; 
//	Te_for_rotM << external_torque.x, external_torque.y, external_torque.z; 
	Fe =  Rotz*Roty*Rotx*Fe_for_rotM;
//	Te =  Rotz*Roty*Rotz*Te_for_rotM;
	
	X_e_x1_dot=-D/M*X_e_x1-K/M*X_e_x2+Fe(0);
	X_e_x2_dot=X_e_x1;
	X_e_x1+=X_e_x1_dot*delta_t.count();
	X_e_x2+=X_e_x2_dot*delta_t.count();
	X_e=-1.0/M*X_e_x2;
	
	Y_e_x1_dot=-D/M*Y_e_x1-K/M*X_e_x2+Fe(1);
	Y_e_x2_dot=Y_e_x1;
	Y_e_x1+=Y_e_x1_dot*delta_t.count();
	Y_e_x2+=Y_e_x2_dot*delta_t.count();
	Y_e=-1.0/M*Y_e_x2;
	
	Z_e_x1_dot=-D_z/M_z*Z_e_x1-K_z/M_z*Z_e_x2+Fe(2);
	Z_e_x2_dot=Z_e_x1;
	Z_e_x1+=Z_e_x1_dot*delta_t.count();
	Z_e_x2+=Z_e_x2_dot*delta_t.count();
	Z_e=-1.0/M_z*Z_e_x2;

	T_e_x1_dot=-D_t/M_t*T_e_x1-K_t/M_t*T_e_x2+external_torque.z;
	T_e_x2_dot=T_e_x1;
	T_e_x1+=T_e_x1_dot*delta_t.count();
	T_e_x2+=T_e_x2_dot*delta_t.count();
	T_e=-1.0/M_t*T_e_x2;


	reference_position.x=X_r;
	reference_position.y=Y_r;
	reference_position.z=Z_r;	
	
}
/*
void position_dob(){
	MinvQ_X_x_dot=MinvQ_A*MinvQ_X_x+MinvQ_B*pos.x;
	MinvQ_X_x+=MinvQ_X_x_dot*delta_t.count();
	MinvQ_X_y=MinvQ_C*MinvQ_X_x;
	
	Q_X_x_dot=Q_A*Q_X_x+Q_B*position_dob_m*X_tilde_ddot_d;
	Q_X_x+=Q_X_x_dot*delta_t.count();
	Q_X_y=Q_C*Q_X_x;

	dhat_X_ddot=(MinvQ_X_y(0)-Q_X_y(0))/position_dob_m;


	MinvQ_Y_x_dot=MinvQ_A*MinvQ_Y_x+MinvQ_B*pos.y;
	MinvQ_Y_x+=MinvQ_Y_x_dot*delta_t.count();
	MinvQ_Y_y=MinvQ_C*MinvQ_Y_x;
	
	Q_Y_x_dot=Q_A*Q_Y_x+Q_B*position_dob_m*Y_tilde_ddot_d;
	Q_Y_x+=Q_Y_x_dot*delta_t.count();
	Q_Y_y=Q_C*Q_Y_x;

	dhat_Y_ddot=(MinvQ_Y_y(0)-Q_Y_y(0))/position_dob_m;

	MinvQ_Z_x_dot=MinvQ_A*MinvQ_Z_x+MinvQ_B*pos.z;
	MinvQ_Z_x+=MinvQ_Z_x_dot*delta_t.count();
	MinvQ_Z_y=MinvQ_C*MinvQ_Z_x;
	
	Q_Z_x_dot=Q_A*Q_Z_x+Q_B*position_dob_m*(Z_tilde_ddot_d+g);
	Q_Z_x+=Q_Z_x_dot*delta_t.count();
	Q_Z_y=Q_C*Q_Z_x;

	dhat_Z_ddot=(MinvQ_Z_y(0)-Q_Z_y(0))/position_dob_m;
	
//	X_tilde_r=X_r-dhat_X;
//	Y_tilde_r=Y_r-dhat_Y;
//	Z_tilde_r=Z_r-dhat_Z;

	force_dhat.x=dhat_X_ddot*position_dob_m;
	force_dhat.y=dhat_Y_ddot*position_dob_m;
	force_dhat.z=dhat_Z_ddot*position_dob_m;

	
//	reference_position.x=X_tilde_r;
//	reference_position.y=Y_tilde_r;
//	reference_position.z=Z_tilde_r;
} */
void force_bandpass_filter(){
//	force_dob(); //?????
	X_x_1_dot = -pass_freq_x/Q_factor*X_x_1 - pow(pass_freq_x,2.0)*X_x_2 + force_dhat.x;
	X_x_2_dot = X_x_1;
	X_x_1 += X_x_1_dot * delta_t.count();
        X_x_2 += X_x_2_dot * delta_t.count();
	Y_x = pass_freq_x/Q_factor*X_x_1;

	X_y_1_dot = -pass_freq_y/Q_factor*X_y_1 - pow(pass_freq_y,2.0)*X_y_2 + force_dhat.y;
	X_y_2_dot = X_y_1;
	X_y_1 += X_y_1_dot * delta_t.count();
        X_y_2 += X_y_2_dot * delta_t.count();
	Y_y = pass_freq_y/Q_factor*X_y_1;

	
	X_z_1_dot = -pass_freq_z/Q_factor*X_z_1 - pow(pass_freq_z,2.0)*X_z_2 + force_dhat.z;
	X_z_2_dot = X_z_1;
	X_z_1 += X_z_1_dot * delta_t.count();
        X_z_2 += X_z_2_dot * delta_t.count();
	Y_z = pass_freq_z/Q_factor*X_z_1;

	filtered_force_dhat.x=Y_x;
	filtered_force_dhat.y=Y_y;
	filtered_force_dhat.z=Y_z;
}

void force_highpass_filter(){
	hpf_X_x_dot = -hpf_freq_x*hpf_X_x+force_dhat.x;
	hpf_X_x += hpf_X_x_dot*delta_t.count();
	hpf_Y_x = -hpf_freq_x*hpf_X_x+force_dhat.x;

	hpf_X_y_dot = -hpf_freq_y*hpf_X_y+force_dhat.y;
	hpf_X_y += hpf_X_y_dot*delta_t.count();
	hpf_Y_y = -hpf_freq_y*hpf_X_y+force_dhat.y;

	hpf_X_z_dot = -hpf_freq_z*hpf_X_z+force_dhat.z;
	hpf_X_z += hpf_X_z_dot*delta_t.count();
	hpf_Y_z = -hpf_freq_z*hpf_X_z+force_dhat.z;

	hpf_filtered_force_dhat.x=hpf_Y_x;
	hpf_filtered_force_dhat.y=hpf_Y_y;
	hpf_filtered_force_dhat.z=hpf_Y_z;
}

void force_dob(){
	
	MinvQ_F_X_x_dot=MinvQ_F_A*MinvQ_F_X_x+MinvQ_F_B*imu_lin_acc.x;
	MinvQ_F_X_x+=MinvQ_F_X_x_dot*delta_t.count();
	MinvQ_F_X_y=MinvQ_F_C*MinvQ_F_X_x;
	
	Q_F_X_x_dot=Q_F_A*Q_F_X_x+Q_F_B*F_xd;
	Q_F_X_x+=Q_F_X_x_dot*delta_t.count();
	Q_F_X_y=Q_F_C*Q_F_X_x;

	dhat_F_X=(MinvQ_F_X_y(0)-Q_F_X_y(0));

	MinvQ_F_Y_x_dot=MinvQ_F_A*MinvQ_F_Y_x+MinvQ_F_B*imu_lin_acc.y;
	MinvQ_F_Y_x+=MinvQ_F_Y_x_dot*delta_t.count();
	MinvQ_F_Y_y=MinvQ_F_C*MinvQ_F_Y_x;
	
	Q_F_Y_x_dot=Q_F_A*Q_F_Y_x+Q_F_B*F_yd;
	Q_F_Y_x+=Q_F_Y_x_dot*delta_t.count();
	Q_F_Y_y=Q_F_C*Q_F_Y_x;

	dhat_F_Y=(MinvQ_F_Y_y(0)-Q_F_Y_y(0));

	MinvQ_F_Z_x_dot=MinvQ_F_A*MinvQ_F_Z_x+MinvQ_F_B*imu_lin_acc.z;
	MinvQ_F_Z_x+=MinvQ_F_Z_x_dot*delta_t.count();
	MinvQ_F_Z_y=MinvQ_F_C*MinvQ_F_Z_x;
	
	Q_F_Z_x_dot=Q_F_A*Q_F_Z_x+Q_F_B*F_zd;
	Q_F_Z_x+=Q_F_Z_x_dot*delta_t.count();
	Q_F_Z_y=Q_F_C*Q_F_Z_x;

	dhat_F_Z=(MinvQ_F_Z_y(0)-Q_F_Z_y(0));
	force_dhat.x=dhat_F_X;
	force_dhat.y=dhat_F_Y;
	force_dhat.z=dhat_F_Z;	
}

void torque_dob(){
	MinvQ_T_X_x_dot=MinvQ_T_A*MinvQ_T_X_x+MinvQ_T_B*imu_ang_vel.x;
	MinvQ_T_X_x+=MinvQ_T_X_x_dot*delta_t.count();
	MinvQ_T_X_y=MinvQ_T_C_x*MinvQ_T_X_x;
	
	Q_T_X_x_dot=Q_T_A*Q_T_X_x+Q_T_B*tautilde_r_d;
	Q_T_X_x+=Q_T_X_x_dot*delta_t.count();
	Q_T_X_y=Q_T_C*Q_T_X_x;

	dhat_tau_r=(MinvQ_T_X_y(0)-Q_T_X_y(0));

	MinvQ_T_Y_x_dot=MinvQ_T_A*MinvQ_T_Y_x+MinvQ_T_B*imu_ang_vel.y;
	MinvQ_T_Y_x+=MinvQ_T_Y_x_dot*delta_t.count();
	MinvQ_T_Y_y=MinvQ_T_C_y*MinvQ_T_Y_x;
	
	Q_T_Y_x_dot=Q_T_A*Q_T_Y_x+Q_T_B*tautilde_p_d;
	Q_T_Y_x+=Q_T_Y_x_dot*delta_t.count();
	Q_T_Y_y=Q_T_C*Q_T_Y_x;

	dhat_tau_p=(MinvQ_T_Y_y(0)-Q_T_Y_y(0));

	MinvQ_T_Z_x_dot=MinvQ_T_A*MinvQ_T_Z_x+MinvQ_T_B*imu_ang_vel.z;
	MinvQ_T_Z_x+=MinvQ_T_Z_x_dot*delta_t.count();
	MinvQ_T_Z_y=MinvQ_T_C_z*MinvQ_T_Z_x;
	
	Q_T_Z_x_dot=Q_T_A*Q_T_Z_x+Q_T_B*tautilde_y_d;
	Q_T_Z_x+=Q_T_Z_x_dot*delta_t.count();
	Q_T_Z_y=Q_T_C*Q_T_Z_x;

	dhat_tau_y=(MinvQ_T_Z_y(0)-Q_T_Z_y(0));
	
	torque_dhat.x=dhat_tau_r;
	torque_dhat.y=dhat_tau_p;
	torque_dhat.z=dhat_tau_y;
	
	external_force_z_handle.data=dhat_tau_p/handle_r;
	
}



void external_force_bias_eliminator(){
	if(!measure_external_force_bias){
		if(eliminator_iter_num==0){
			ROS_INFO("Measuring external force bias...");
		}
		
		F_ex_bias_integrator+=external_force.x;
		F_ey_bias_integrator+=external_force.y;
		
		eliminator_iter_num++;
	}
	if(eliminator_iter_num==eliminator_iter_max){
		F_ex_bias = F_ex_bias_integrator/(double)eliminator_iter_max;
		F_ey_bias = F_ey_bias_integrator/(double)eliminator_iter_max;
		eliminator_iter_num++;
		measure_external_force_bias=true;
		ROS_INFO("Measuring complete!!!");
	}		
	
}

void mass_update()
{
	mass_x_dot=-mass_lpf_fc*mass_x+external_force.z/g;
	mass_x+=mass_x_dot*delta_t.count();
	mass_y=mass_lpf_fc*mass_x;

	mass_topic.data=mass+mass_y;

}

void inch_sineGenerator()
{
	cnt = cnt+1;
	inch_sine = inch_Amp * sin(cnt / inch_period / freq * 2 * pi + pi / 2) - inch_Amp;
	//ROS_ERROR("SINE!!!");
}



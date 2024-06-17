#include "ros/ros.h"
#include "cmath"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <geometry_msgs/Twist.h>

#define PI 3.1415926535
double freq = 200;

bool command_Flag = false;

Eigen::Vector2d End_Effector_Position_cmd;
Eigen::Vector2d End_Effector_Position_meas;

Eigen::Vector2d angle_cmd;
Eigen::Vector2d angle_meas;
Eigen::Vector2d angle_safe;

Eigen::Vector2d angle_max;
Eigen::Vector2d angle_min;
// |----0-----0-----|
// |                |
// 0                |
// |                |
// -                -
Eigen::Matrix2d D1;
Eigen::Matrix2d D2;
Eigen::Matrix2d D3;
Eigen::Matrix2d D4;

double length_1 = 0.22;
double length_2 = 0.22;

void initParameters()
{
    angle_max << 10, 10;
    angle_min << -10, -10;
}


void EE_cmd_gui_Callback(const geometry_msgs::Twist &msg)
{
    command_Flag = true;
    //GUI에서 받아오는 End Effector Command Callback
    End_Effector_Position_cmd[0] = msg.linear.x;
    End_Effector_Position_cmd[1] = msg.linear.y;
}


void dynamixel_angle_Callback(const sensor_msgs::JointState &msg)
{
    angle_meas[0] = msg.position.at(0);
    angle_meas[1] = msg.position.at(1);
}

Eigen::Vector2d Forward_Kinematics(Eigen::Vector2d angle_meas)
{
    //측정값으로부터 FK 풀기
    Eigen::Vector2d End_Effector_Position_meas_;
    End_Effector_Position_meas_[0] = length_1*cos(angle_meas[0]) + length_2*cos(angle_meas[0] + angle_meas[1]);
    End_Effector_Position_meas_[1] = length_1*sin(angle_meas[0]) + length_2*sin(angle_meas[0] + angle_meas[1]);

    return End_Effector_Position_meas_;

}

Eigen::Vector2d Inverse_Kinematics(Eigen::Vector2d End_Effector_Position_cmd)
{
    //GUI에서 받아온 Cmd로 IK 풀기
    Eigen::Vector2d angle_cmd_;
    angle_cmd_[1] = acos((pow(End_Effector_Position_cmd[0],2) +  pow(End_Effector_Position_cmd[1],2) - pow(length_1,2) - pow(length_2,2)) / (2*length_1*length_2));
    angle_cmd_[0] = atan(End_Effector_Position_cmd[1] / End_Effector_Position_cmd[0]) - atan((length_2 * sin(angle_cmd_[1])/(length_1 + length_2 * cos(angle_cmd_[1]))));

    return angle_cmd_;
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle nh;
    ros::Rate rate(freq);

    ros::Publisher goal_dynamixel_position_pub_ = nh.advertise<sensor_msgs::JointState>("/goal_dynamixel_position", 10);
    ros::Publisher measured_EE_position_pub_ = nh.advertise<geometry_msgs::Twist>("/inch/EE_meas", 10);

	ros::Subscriber End_Effector_Position_cmd_sub_ = nh.subscribe("/inch/EE_cmd_gui",100,EE_cmd_gui_Callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber dynamixel_angle_sub_ = nh.subscribe("/joint_states", 100, dynamixel_angle_Callback, ros::TransportHints().tcpNoDelay());
    initParameters();

    geometry_msgs::Twist EE_meas;

    while(ros::ok())
    {
        sensor_msgs::JointState command_angle;
        command_angle.header.stamp = ros::Time::now();


        angle_cmd = Inverse_Kinematics(End_Effector_Position_cmd);
        angle_safe = Angle_Safe_Function(angle_cmd);
        End_Effector_Position_meas = Forward_Kinematics(angle_meas);

        command_angle.position.push_back(angle_safe[0]);
        command_angle.position.push_back(angle_safe[1]);
        command_angle.position.push_back(0);

        EE_meas.linear.x = End_Effector_Position_meas[0];
        EE_meas.linear.y = End_Effector_Position_meas[1];

        if(command_Flag) goal_dynamixel_position_pub_.publish(command_angle);
        measured_EE_position_pub_.publish(EE_meas);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}





// Eigen::MatrixXd DH_X(double theta, double d)
// {
//     Eigen::MatrixXd DH_Matrix;

//     DH_Matrix << 1, 0,           0,          d,
//                  0, cos(theta), -sin(theta), 0,
//                  0, sin(theta),  cos(theta), 0,
//                  0, 0,           0,          1;

//     return DH_Matrix;
// }

// Eigen::MatrixXd DH_Y(double theta, double d)
// {
//     Eigen::MatrixXd DH_Matrix;

//     DH_Matrix << cos(theta),  0, sin(theta), 0,
//                  0,           1, 0,          d,
//                  -sin(theta), 0, cos(theta), 0,
//                  0,           0,          0, 1;

//     return DH_Matrix;
// }

// Eigen::MatrixXd DH_Z(double theta, double d)
// {
//     Eigen::MatrixXd DH_Matrix;

//     DH_Matrix << cos(theta), -sin(theta), 0, 0,
//                  sin(theta),  cos(theta), 0, 0,
//                  0,           0,          1, d,
//                  0,           0,          0, 1;

//     return DH_Matrix;
// }

// Eigen::Matrix3d DH_X(double theta, double d)
// {
//     Eigen::Matrix3d DH_Matrix;

//     DH_Matrix << cos(theta), -sin(theta), 0,
//                  sin(theta),  cos(theta), d,
//                  0,           0,          1;

//     return DH_Matrix;
// }
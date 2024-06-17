#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>


int main(int argc, char** argv)
{
    ros::init(argc,argv,"setting_link_1_listener");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer_globalEE;
    tf2_ros::TransformListener tfListener_globalEE(tfBuffer_globalEE);

    geometry_msgs::PoseStamped global_frame;
    geometry_msgs::Twist Link1_pos;

    double roll, pitch, yaw;

    ros::Publisher Link1_Pub_ = nh.advertise<geometry_msgs::Twist>("/inch/global_link1", 10);

    ros::Rate rate(120);

    while(nh.ok())
    {
        geometry_msgs::TransformStamped transformStamped_globalEE;

        try
        {
            transformStamped_globalEE = tfBuffer_globalEE.lookupTransform("tf/inch/paletrone", "tf/inch/Link1", ros::Time(0));
        }
        catch(tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // For /dasom/tf/global_EE_pose
        global_frame.pose.position.x = transformStamped_globalEE.transform.translation.x;
        global_frame.pose.position.y = transformStamped_globalEE.transform.translation.y;
        global_frame.pose.position.z = transformStamped_globalEE.transform.translation.z;

        global_frame.pose.orientation.x = transformStamped_globalEE.transform.rotation.x;
        global_frame.pose.orientation.y = transformStamped_globalEE.transform.rotation.y;
        global_frame.pose.orientation.z = transformStamped_globalEE.transform.rotation.z;
        global_frame.pose.orientation.w = transformStamped_globalEE.transform.rotation.w;

        Link1_pos.linear = transformStamped_globalEE.transform.translation;

        tf::Quaternion quat;
        tf::quaternionMsgToTF(global_frame.pose.orientation, quat);

        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        Link1_pos.angular.x = roll;
        Link1_pos.angular.y = pitch;
        Link1_pos.angular.z = yaw;
        
        Link1_Pub_.publish(Link1_pos);

        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}



#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include "tf/transform_datatypes.h"


#define PI 3.141592

geometry_msgs::Twist Link_angle;


void paletroneCallback(geometry_msgs::PoseStamped msg)
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "tf/inch/paletrone";
    transformStamped.transform.translation.x = msg.pose.position.x;
    transformStamped.transform.translation.y = msg.pose.position.y;
    transformStamped.transform.translation.z = msg.pose.position.z;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.orientation, quat);

    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();



    br.sendTransform(transformStamped);
}


void Link1Callback(geometry_msgs::PoseStamped msg)
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "tf/inch/Link1";
    transformStamped.transform.translation.x = msg.pose.position.x;
    transformStamped.transform.translation.y = msg.pose.position.y;
    transformStamped.transform.translation.z = msg.pose.position.z;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.orientation, quat);

    tf::Matrix3x3(quat).getRPY(Link_angle.linear.x, Link_angle.linear.y, Link_angle.linear.z);

    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();



    br.sendTransform(transformStamped);
}

void Link2Callback(geometry_msgs::PoseStamped msg)
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "tf/inch/Link2";
    transformStamped.transform.translation.x = msg.pose.position.x;
    transformStamped.transform.translation.y = msg.pose.position.y;
    transformStamped.transform.translation.z = msg.pose.position.z;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.orientation, quat);

    tf::Matrix3x3(quat).getRPY(Link_angle.angular.x, Link_angle.angular.y, Link_angle.angular.z);

    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();



    br.sendTransform(transformStamped);
}


int main(int argc, char **argv){
    ros::init(argc,argv,"setting_Link_tf");

    ros::NodeHandle nh;
    ros::Subscriber paletrone_sub_ = nh.subscribe("/inchbase/world", 10, paletroneCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber Link1_sub_ = nh.subscribe("/inchLink1/world", 10, Link1Callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber Link2_sub_ = nh.subscribe("/inchLink2/world", 10, Link2Callback, ros::TransportHints().tcpNoDelay());
    ros::Publisher Link_Pub_ = nh.advertise<geometry_msgs::Twist>("/inch/Link_angle/global", 10);
    ros::Rate loop(100);

    while(ros::ok())
    {
    Link_Pub_.publish(Link_angle);
    loop.sleep();
    ros::spinOnce();
    }return 0;
}

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"inch_tf2_listener");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    
    geometry_msgs::PoseStamped global_frame;

    ros::Publisher listner_publisher = nh.advertise<geometry_msgs::PoseStamped>("/inch/tf/tf_listener", 10);

    ros::Rate rate(120);

    while(nh.ok())
    {
        geometry_msgs::TransformStamped transformStamped;

        try
        {
            transformStamped = tfBuffer.lookupTransform("world", "tf/global_EE_pose", ros::Time(0));
        }
        catch(tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        global_frame.pose.position.x = transformStamped.transform.translation.x;
        global_frame.pose.position.y = transformStamped.transform.translation.y;
        global_frame.pose.position.z = transformStamped.transform.translation.z;

        global_frame.pose.orientation.x = transformStamped.transform.rotation.x;
        global_frame.pose.orientation.y = transformStamped.transform.rotation.y;
        global_frame.pose.orientation.z = transformStamped.transform.rotation.z;
        global_frame.pose.orientation.w = transformStamped.transform.rotation.w;

        listner_publisher.publish(global_frame);

        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
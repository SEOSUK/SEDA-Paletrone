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
    
    geometry_msgs::Twist gimbal_EE_cmd;

    ros::Publisher gimbal_EE_cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/inch/gimbal_EE_ref", 10);


    ros::Rate rate(120);

    while(nh.ok())
    {
        geometry_msgs::TransformStamped transformStamped;

        try
        {
            transformStamped = tfBuffer.lookupTransform("tf/inch/Base", "tf/inch/EE_gimbal_tf", ros::Time(0));
        }
        catch(tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        gimbal_EE_cmd.linear.x = 0;
        gimbal_EE_cmd.linear.y = transformStamped.transform.translation.y;
        gimbal_EE_cmd.linear.z = transformStamped.transform.translation.z;


        gimbal_EE_cmd_pub_.publish(gimbal_EE_cmd);

        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}

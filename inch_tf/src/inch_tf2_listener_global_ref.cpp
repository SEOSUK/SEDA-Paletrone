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
    ros::init(argc,argv,"inch_tf_global_EE_ref");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    
    geometry_msgs::Twist global_EE_ref;

    ros::Publisher global_EE_ref_pub_ = nh.advertise<geometry_msgs::Twist>("/inch/global_EE_ref", 10);


    ros::Rate rate(120);

    while(nh.ok())
    {
        geometry_msgs::TransformStamped transformStamped;

        try
        {
            transformStamped = tfBuffer.lookupTransform("world", "tf/inch/EE_global_ref", ros::Time(0));
        }
        catch(tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        global_EE_ref.linear.x = 0;
        global_EE_ref.linear.y = transformStamped.transform.translation.y;
        global_EE_ref.linear.z = transformStamped.transform.translation.z;


        global_EE_ref_pub_.publish(global_EE_ref);

        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}

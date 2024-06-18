#include "../include/inch_tf2_broadcaster.h"

TFBroadcaster::TFBroadcaster()
: node_handle_("")
{
  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initSubscriber();

  inch_roll_lpf = new InchMisc();


}

TFBroadcaster::~TFBroadcaster()
{
  ROS_INFO("Bye TFBroadcaster!");
  ros::shutdown();
}


void TFBroadcaster::palletroneOptitrackCallback(const geometry_msgs::PoseStamped& msg)
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "tf/palletrone_optitrack";
    transformStamped.transform.translation.x = msg.pose.position.x;
    transformStamped.transform.translation.y = msg.pose.position.y;
    transformStamped.transform.translation.z = msg.pose.position.z;

    transformStamped.transform.rotation.x = msg.pose.orientation.x;
    transformStamped.transform.rotation.y = msg.pose.orientation.y;
    transformStamped.transform.rotation.z = msg.pose.orientation.z;
    transformStamped.transform.rotation.w = msg.pose.orientation.w;

    br.sendTransform(transformStamped);
}


void TFBroadcaster::initPublisher()
{

}

void TFBroadcaster::initSubscriber()
{                                          
    palletrone_optitrack_sub_ = node_handle_.subscribe("/inchPalletrone/world", 10, &TFBroadcaster::palletroneOptitrackCallback, this, ros::TransportHints().tcpNoDelay());
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"inch_tf2_broadcaster");
    ros::NodeHandle nh;

    TFBroadcaster tf_br_;

    ros::Rate loop(120);



    tf_br_.time_i = ros::Time::now().toSec();
    tf_br_.time_f = 0;
    tf_br_.time_loop = 0;

    while(ros::ok())
    {
        tf_br_.time_f = ros::Time::now().toSec();
        tf_br_.time_loop = tf_br_.time_f - tf_br_.time_i;
        tf_br_.time_i = ros::Time::now().toSec();

        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}




//        quat to rpy
//        tf::Quaternion quat;
//        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

//        rpy to quat
//        tf::Quaternion quat;
//        tf::quaternionMsgToTF(msg.pose.orientation, quat);
//        quat.setRPY(roll, pitch, yaw);

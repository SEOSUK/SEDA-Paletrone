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


void TFBroadcaster::inch_EE_meas_callback(const geometry_msgs::Twist& msg)
{

  /*****************************************************************************
  ** Body End Effector
  *****************************************************************************/
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "tf/inch/EE_meas";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = msg.linear.y;
    transformStamped.transform.translation.z = msg.linear.z;

    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;

    br.sendTransform(transformStamped);


  /*****************************************************************************
  ** Global End Effector
  *****************************************************************************/
    static tf2_ros::StaticTransformBroadcaster br_global;
  
    geometry_msgs::TransformStamped transformStamped_global;

    transformStamped_global.header.stamp = ros::Time::now();
    transformStamped_global.header.frame_id = "tf/inch/Base";
    transformStamped_global.child_frame_id = "tf/inch/EE_global_meas";
    transformStamped_global.transform.translation.x = 0;
    transformStamped_global.transform.translation.y = msg.linear.y;
    transformStamped_global.transform.translation.z = msg.linear.z;

    transformStamped_global.transform.rotation.x = 0;
    transformStamped_global.transform.rotation.y = 0;
    transformStamped_global.transform.rotation.z = 0;
    transformStamped_global.transform.rotation.w = 1;

    br_global.sendTransform(transformStamped_global);

  /*****************************************************************************
  ** Gimballing
  //  1. BroadCast: Topic (/inch/EE_meas)을 받아서 tf/inch/Base to tf/inch/EE_gimbal_cmd.
  //  2. Listener: Topic (/inch/gimbal_EE_cmd)을 쏜다. tf/inch/Base to tf/inch/EE_gimbal_cmd로  하 ㅅㅂ 아닌거같은데
  //  3. 일단 짠다
  *****************************************************************************/
    if(!gimbal_Flag) 
    {
      static tf2_ros::StaticTransformBroadcaster br_gimbal;
    
      geometry_msgs::TransformStamped transformStamped_gimbal;

      transformStamped_gimbal.header.stamp = ros::Time::now();
      transformStamped_gimbal.header.frame_id = "tf/inch/gimbal_Base";
      transformStamped_gimbal.child_frame_id = "tf/inch/EE_gimbal_tf";
      transformStamped_gimbal.transform.translation.x = 0;
      transformStamped_gimbal.transform.translation.y = msg.linear.y;
      transformStamped_gimbal.transform.translation.z = msg.linear.z;

      transformStamped_gimbal.transform.rotation.x = 0;
      transformStamped_gimbal.transform.rotation.y = 0;
      transformStamped_gimbal.transform.rotation.z = 0;
      transformStamped_gimbal.transform.rotation.w = 1;

        br_gimbal.sendTransform(transformStamped_gimbal);
      }
}

void TFBroadcaster::inch_EE_cmd_callback(const geometry_msgs::Twist& msg)
{
  /*****************************************************************************
  ** Body  End Effector
  *****************************************************************************/
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "tf/inch/EE_cmd";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = msg.linear.y;
    transformStamped.transform.translation.z = msg.linear.z;

    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;

    br.sendTransform(transformStamped);


  /*****************************************************************************
  ** Global End Effector   (Gimballing AutoChange)
  *****************************************************************************/
    static tf2_ros::StaticTransformBroadcaster br_global;
  
    geometry_msgs::TransformStamped transformStamped_global;

    transformStamped_global.header.stamp = ros::Time::now();
    transformStamped_global.header.frame_id = "tf/inch/Base";
    transformStamped_global.child_frame_id = "tf/inch/EE_global_cmd";
    transformStamped_global.transform.translation.x = 0;
    transformStamped_global.transform.translation.y = msg.linear.y;
    transformStamped_global.transform.translation.z = msg.linear.z;

    transformStamped_global.transform.rotation.x = 0;
    transformStamped_global.transform.rotation.y = 0;
    transformStamped_global.transform.rotation.z = 0;
    transformStamped_global.transform.rotation.w = 1;

    br_global.sendTransform(transformStamped_global);
}

void TFBroadcaster::inch_Palletrone_callback(const geometry_msgs::PoseStamped& msg)
{
  /*****************************************************************************
  ** Global End Effector
  *****************************************************************************/
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "tf/inch/Base";
    transformStamped.transform.translation.x = msg.pose.position.x;
    transformStamped.transform.translation.y = msg.pose.position.y;
    transformStamped.transform.translation.z = msg.pose.position.z;

    transformStamped.transform.rotation.x = msg.pose.orientation.x;
    transformStamped.transform.rotation.y = msg.pose.orientation.y;
    transformStamped.transform.rotation.z = msg.pose.orientation.z;
    transformStamped.transform.rotation.w = msg.pose.orientation.w;

    br.sendTransform(transformStamped);

  ROS_FATAL("TF not gimbal mode");    

  /*****************************************************************************
  ** Global End Effector
  *****************************************************************************/
  if(!gimbal_Flag) 
  {
    static tf2_ros::StaticTransformBroadcaster br_gimbal_base;
    geometry_msgs::TransformStamped transformStamped_gimbal_base;

    transformStamped_gimbal_base.header.stamp = ros::Time::now();
    transformStamped_gimbal_base.header.frame_id = "world";
    transformStamped_gimbal_base.child_frame_id = "tf/inch/gimbal_Base";
    transformStamped_gimbal_base.transform.translation.x = msg.pose.position.x;
    transformStamped_gimbal_base.transform.translation.y = msg.pose.position.y;
    transformStamped_gimbal_base.transform.translation.z = msg.pose.position.z;

    transformStamped_gimbal_base.transform.rotation.x = msg.pose.orientation.x;
    transformStamped_gimbal_base.transform.rotation.y = msg.pose.orientation.y;
    transformStamped_gimbal_base.transform.rotation.z = msg.pose.orientation.z;
    transformStamped_gimbal_base.transform.rotation.w = msg.pose.orientation.w;

    br_gimbal_base.sendTransform(transformStamped_gimbal_base);    
  ROS_WARN("TF gimbal mode");
  }
}


bool TFBroadcaster::gimbal_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  if (gimbal_Flag)
  {
    ROS_INFO("TF not gimbal mode");
    gimbal_Flag = false;
  }
  else
  {
    ROS_INFO("TF gimbaling");
    gimbal_Flag = true;
  }

  return true;
}

void TFBroadcaster::initPublisher()
{
//  Link1_pub_ = node_handle_.advertise<geometry_msgs::Vector3>("/inch/tf/link1", 10);
}

void TFBroadcaster::initSubscriber()
{
    inch_EE_meas_sub_ = node_handle_.subscribe("/inch/EE_meas", 10, &TFBroadcaster::inch_EE_meas_callback, this, ros::TransportHints().tcpNoDelay());             
    inch_EE_cmd_sub_ = node_handle_.subscribe("/inch/EE_ref", 10, &TFBroadcaster::inch_EE_cmd_callback, this, ros::TransportHints().tcpNoDelay());             
    inch_BasePlate_sub_ = node_handle_.subscribe("/inchBase/world", 10, &TFBroadcaster::inch_Palletrone_callback, this, ros::TransportHints().tcpNoDelay());      // From Optitrack
    inch_gimbal_Flag_server_ = node_handle_.advertiseService("/inch/gimbalSrv", &TFBroadcaster::gimbal_callback, this);
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

// void TFBroadcaster::inch_EE_meas_callback(const geometry_msgs::Twist& msg)
// {
//     static tf2_ros::StaticTransformBroadcaster br;
//     geometry_msgs::TransformStamped transformStamped;

//     transformStamped.header.stamp = ros::Time::now();
//     transformStamped.header.frame_id = "world";
//     transformStamped.child_frame_id = "tf/inchLink1";
//     transformStamped.transform.translation.x = msg.pose.position.x;
//     transformStamped.transform.translation.y = msg.pose.position.y;
//     transformStamped.transform.translation.z = msg.pose.position.z;

//     transformStamped.transform.rotation.x = msg.pose.orientation.x;
//     transformStamped.transform.rotation.y = msg.pose.orientation.y;
//     transformStamped.transform.rotation.z = msg.pose.orientation.z;
//     transformStamped.transform.rotation.w = msg.pose.orientation.w;

//     br.sendTransform(transformStamped);


//     tf::Quaternion quat;
//     tf::quaternionMsgToTF(msg.pose.orientation, quat);
//     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

//   //  Link1_pub_.publish(Link1_msg);
// }

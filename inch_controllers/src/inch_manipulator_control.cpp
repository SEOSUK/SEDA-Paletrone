#include "inch_controllers/inch_manipulator_control.h"

InchControl::InchControl()
: node_handle_(""), priv_node_handle_("~")
{

  /************************************************************
  ** Launch file parameters
  ************************************************************/  
  length_1 = node_handle_.param<double>("length_1", 0);
  length_2 = node_handle_.param<double>("length_2", 0);
  length_3 = node_handle_.param<double>("length_3", 0);

  ROS_INFO("length [%lf] [%lf] [%lf]", length_1, length_2, length_3);

  /************************************************************
  ** class init
  ************************************************************/  

  inch_jnt1_ = new InchJoint(); 
  inch_jnt2_ = new InchJoint(); 
  inch_jnt3_ = new InchJoint(); 

  inch_EE_1_misc_ = new InchMisc(); //InchJoint 1에 반영할 inch_misc
  inch_EE_2_misc_ = new InchMisc(); //InchJoint 2에 반영할 inch_misc



  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initPublisher();
  initSubscriber();
  initServer();

  ROS_INFO("Inch Manipulator Control node start");


  // Initializer
  // 처음에 딱 한 번 발동했음 하는 것들 모음
  time_i = ros::Time::now().toSec();
  test_msg.data.resize(10);



  //Init Butterworth 2nd
  inch_EE_1_misc_->init_butterworth_2nd_filter(10);
  inch_EE_2_misc_->init_butterworth_2nd_filter(1.);



}


InchControl::~InchControl()
{
  // 노드 꺼질 때 한 번 작동작동

  deleteToolbox();

  ROS_INFO("Bye!");
  ros::shutdown();
}


void InchControl::initPublisher()
{
  theta_command_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/inch/goal_dynamixel_position", 10); // directly to dynamixel
  EE_meas_pub_ = node_handle_.advertise<geometry_msgs::Twist>("/inch/EE_meas", 10);

  // Tester publisher
  test_pub_ = node_handle_.advertise<std_msgs::Float64MultiArray>("/inch/test_Pub", 10);
}

void InchControl::initSubscriber()
{

}

void InchControl::initServer()
{

}

void InchControl::PublishData()
{
  //inch/goal_dynamixel_position
  //Operate Dynamixel
  theta_command_msg.position.push_back(theta_ref[0]);
  theta_command_msg.position.push_back(theta_ref[1]);
  theta_command_pub_.publish(theta_command_msg);
  

  //inch/EE_meas
  //End Effector position from FK
  EE_meas_msg.linear.x = EE_cmd[0];
  EE_meas_msg.linear.y = EE_cmd[1];
  EE_meas_pub_.publish(EE_meas_msg);


  //inch/test_Pub
  //Just test
  test_msg.data[0] = EE_cmd[0];  //FK IK Cross Check
  test_msg.data[1] = EE_meas[0];
  test_msg.data[2] = EE_cmd[1];
  test_msg.data[3] = EE_meas[1]; 

  test_pub_.publish(test_msg);
}

void InchControl::deleteToolbox()
{
  delete inch_jnt1_;
  delete inch_jnt2_;
  delete inch_jnt3_;
  
  delete inch_EE_1_misc_;
  delete inch_EE_2_misc_;
}

void InchControl::TimeCount()
{
    time_f = ros::Time::now().toSec();
    time_loop = time_f - time_i; // dt 입니다.
    time_i = ros::Time::now().toSec();
    time_real = time_real +  time_loop; // 노드 켜고 경과된 시간 입니다.
}

void InchControl::SolveInverseForwardKinematics()
{
  theta_ref = InverseKinematics_2dof(EE_cmd);

  EE_meas = ForwardKinematics_2dof(theta_ref);

}

void InchControl::Trajectory_mode()
{
  // 여기서 작동 모드 (EE command를 어디서 받을지) 결정

  if (true) Test_trajectory_generator_2dof();
  else if (true) trajectory_gimbaling();
}


void InchControl::Test_trajectory_generator_2dof()
{
  EE_cmd[0] = 0.01 * sin(2 * PI * 0.1 * time_real) + 0.15; // Amp * sin (2 pi f t) + bias
  EE_cmd[1] = 0.01 * cos(2 * PI * 0.1 * time_real) + 0.2 + 0.1 * sin (time_loop * PI * PI);  // Amp * sin (2 pi f t) + bias + noise 
}

void InchControl::trajectory_gimbaling()
{
  ROS_INFO("Happy Gimbaling YYEEEAAAHHH!");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "InchControl");
  InchControl inch_ctrl_;


  ros::Rate loop_rate(200);

  while (ros::ok())
  {
    inch_ctrl_.TimeCount();


    //Functions for Control!! - start;
    inch_ctrl_.Trajectory_mode();

    inch_ctrl_.SolveInverseForwardKinematics();



    //Functions for Control!! - End

    inch_ctrl_.PublishData();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

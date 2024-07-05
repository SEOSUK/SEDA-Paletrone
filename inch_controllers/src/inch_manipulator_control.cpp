#include "inch_controllers/inch_manipulator_control.h"

InchControl::InchControl()
: node_handle_(""), priv_node_handle_("~")
{
  /************************************************************
  ** Launch file parameters
  ************************************************************/  

  robot_name_ = node_handle_.param<std::string>("robot_name", "inch"); 
  Link1_length = node_handle_.param<double>("Link1_length", 0);
  Link2_length = node_handle_.param<double>("Link2_length", 0);

  Link1_COM = node_handle_.param<double>("Link1_COM", 0);
  Link2_COM = node_handle_.param<double>("Link2_COM", 0);

  Link1_mass = node_handle_.param<double>("Link1_mass", 0);
  Link2_mass = node_handle_.param<double>("Link2_mass", 0);





  /************************************************************
  ** class init
  ************************************************************/  

  inch_joint = new InchJoint(); 

  inch_q_meas_butterworth = new InchMisc(); //qdot에 butterworth 넣을거임

  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initPublisher();
  initSubscriber();
  initServer();

  ROS_WARN("===================================");
  ROS_WARN("Inch Manipulator Control node start");
  ROS_WARN("===================================");


  // Initializer
  // 처음에 딱 한 번 발동했음 하는 것들 모음
  time_i = ros::Time::now().toSec();
  test_msg.data.resize(10);



  //Init Butterworth 2nd
  inch_q_meas_butterworth->init_butterworth_2nd_filter(40);


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
  theta_command_pub_ = node_handle_.advertise<sensor_msgs::JointState>("goal_dynamixel_position", 10); // directly to dynamixel
  EE_meas_pub_ = node_handle_.advertise<geometry_msgs::Twist>(robot_name_ + "/EE_meas", 10);

  // Tester publisher
  test_pub_ = node_handle_.advertise<std_msgs::Float64MultiArray>(robot_name_ + "test_Pub", 10);
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
  sensor_msgs::JointState theta_command_msg;

  theta_command_msg.position.push_back(theta_cmd[0]);
  theta_command_pub_.publish(theta_command_msg);


  //inch/EE_meas
  //End Effector position from FK
  EE_meas_msg.linear.x = EE_cmd[0];
  EE_meas_msg.linear.y = EE_cmd[1];
  EE_meas_pub_.publish(EE_meas_msg);


  //inch/test_Pub
  //Just test
  test_msg.data[0] = q_ref[0]; // 레퍼런스
  test_msg.data[1] = q_cmd[0]; // 어드미턴스 통과 레퍼런스
  test_msg.data[2] = q_ref[0] - inch_joint->q_meas[0]; // 에러
  test_msg.data[3] = theta_cmd[0]; // 서보커맨드
  test_msg.data[4] = inch_joint->phi_meas[0]; // 서보커맨드


  test_pub_.publish(test_msg);
}

void InchControl::deleteToolbox()
{
  delete inch_joint;
  
  delete inch_q_meas_butterworth;
  delete inch_link1_PID;
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
  q_cmd = InverseKinematics_2dof(EE_cmd);

  EE_meas = ForwardKinematics_2dof(q_cmd);

}

void InchControl::Trajectory_mode()
{
  // 여기서 작동 모드 (EE command를 어디서 받을지) 결정

  if (true) Test_trajectory_generator_2dof();
  else if (true) trajectory_gimbaling();
}


void InchControl::Test_trajectory_generator_2dof()
{
   q_ref[0] = 0.6 * sin (2 *PI * 0.3 * time_real) + 0.3; // sine wave

//else time_real = 0;
  
}

void InchControl::trajectory_gimbaling()
{
  ROS_INFO("Happy Gimbaling YYEEEAAAHHH!");
}


void InchControl::F_ext_processing()
{
  F_ext = ForceEstimation(inch_joint->q_meas, inch_joint->tau_ext);

}

void InchControl::Experiment_0623_1Link()
{




/*
  //MPC Solver [in: r,q,q_dot | out: v(q_ddot)]
  double k1 = 15.966;
  double k2 = 5.65;
  double k3 = k1;
  double v = k3*theta_ref[0]-k1*q_meas[0]-k2*q_meas_dot[0];   


  //(Feedback Linearization)^-1 [in: v,q,q_dot,u_dot | out: theta_cmd]

  //model parameter
  double m=0.2;
  double l=0.5+0.05;
  double g=9.81;
  double k=1000;
  double d=0.02525;
  double c=0.0; //Note* this code is """No damper Case""

  double tau=m*l*l*v/3+0.5*m*g*l*cos(q_meas[0]);
  
  theta_cmd[0] = q_meas[0] - (tau)/(-4*k*d*d); //u = q-phi

  //ROS_WARN("cmd: [%lf]    ref:[%lf]", theta_cmd[0], theta_ref[0]);

  if(theta_cmd[0] > 90 * PI / 180) 
  {
    theta_cmd[0] = 90 * PI / 180;
    ROS_FATAL("over 90!!!");
  }
  if(theta_cmd[0] < -5 * PI / 180)
  {
    theta_cmd[0] = 0;
    ROS_FATAL("under 0!!!");
  } 
*/

}

void InchControl::YujinInit()
{


}

void InchControl::YujinWhile()
{


}

void InchControl::HanryungInit()
{


}

void InchControl::HanryungWhile()
{

  
}

void InchControl::SeukInit()
{

  inch_link1_PID = new InchMisc(); // link1에 pid 쓸거임

  inch_link1_PID->init_PID_controller(1, 0.001, 0., 40);
  init_Admittance(0.1, 0.5, 0.5);


}

void InchControl::SeukWhile()
{
  //Let's define Parameters~~
  double k_spring = 1;

  // end.
  



  //command generation
  q_ref[0] = 45 * PI / 180 * sin (2 * PI * 0.1* time_real) + 45 * PI / 180 ; // sine wave


  //admittance
  q_ref[0] = admittanceControly(q_ref[0], -k_spring * inch_joint->phi_meas[0], time_loop);
  
  //PID
  theta_cmd[0] = q_ref[0] + inch_link1_PID->PID_controller(q_ref[0] - inch_joint->q_meas[0], time_loop);
  

  //Saturation
  if (theta_cmd[0] > 90*PI/180) 
  {
    ROS_ERROR("OVER, %lf", theta_cmd[0]);
    theta_cmd[0] = 90*PI/180;
  }
  else if (theta_cmd[0] < -10*PI/180) 
  {
    ROS_ERROR("UNDER, %lf", theta_cmd[0]);
    theta_cmd[0] = -10*PI/180;
  }

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "InchControl");
  InchControl inch_ctrl_;

//  inch_ctrl_.YujinInit();
//  inch_ctrl_.HanryungInit();
  inch_ctrl_.SeukInit();

  ros::Rate loop_rate(200);

  while (ros::ok())
  {
    inch_ctrl_.TimeCount();

  //  inch_ctrl_.YujinWhile();
  //  inch_ctrl_.HanryungWhile();
    inch_ctrl_.SeukWhile();

    inch_ctrl_.PublishData();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

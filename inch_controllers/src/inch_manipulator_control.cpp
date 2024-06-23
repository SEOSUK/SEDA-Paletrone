#include "inch_controllers/inch_manipulator_control.h"

InchControl::InchControl()
: node_handle_(""), priv_node_handle_("~")
{
  ros::Rate init_sleep(0.2);
  init_sleep.sleep();

  /************************************************************
  ** Launch file parameters
  ************************************************************/  

  robot_name_ = node_handle_.param<std::string>("robot_name", "inch"); 
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

  ROS_WARN("===================================");
  ROS_WARN("Inch Manipulator Control node start");
  ROS_WARN("===================================");


  // Initializer
  // 처음에 딱 한 번 발동했음 하는 것들 모음
  time_i = ros::Time::now().toSec();
  test_msg.data.resize(10);



  //Init Butterworth 2nd
  inch_EE_1_misc_->init_butterworth_2nd_filter(40);
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
  theta_command_pub_ = node_handle_.advertise<sensor_msgs::JointState>("goal_dynamixel_position", 10); // directly to dynamixel
  EE_meas_pub_ = node_handle_.advertise<geometry_msgs::Twist>(robot_name_ + "/EE_meas", 10);

  // Tester publisher
  test_pub_ = node_handle_.advertise<std_msgs::Float64MultiArray>(robot_name_ + "test_Pub", 10);
}

void InchControl::initSubscriber()
{
  dynamixel_workbench_sub_ = node_handle_.subscribe("/joint_states", 10, &InchControl::dynamixel_workbench_callback, this, ros::TransportHints().tcpNoDelay());
  Optitrack_sub_ = node_handle_.subscribe("/inch/tf/link1", 10, &InchControl::Optitrack_callback, this, ros::TransportHints().tcpNoDelay());
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

  ROS_INFO("aaaaaaaaaaa[%lf] [%lf]", theta_cmd[0], theta_ref[0]);


  //inch/EE_meas
  //End Effector position from FK
  EE_meas_msg.linear.x = EE_cmd[0];
  EE_meas_msg.linear.y = EE_cmd[1];
  EE_meas_pub_.publish(EE_meas_msg);


  //inch/test_Pub
  //Just test
  test_msg.data[0] = theta_ref[0];   //Sine wave
  test_msg.data[1] = theta_cmd[0];
  test_msg.data[2] = theta_meas[0];
  test_msg.data[3] = q_meas[0];
  

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
  theta_cmd = InverseKinematics_2dof(EE_cmd);

  EE_meas = ForwardKinematics_2dof(theta_cmd);

}

void InchControl::Trajectory_mode()
{
  // 여기서 작동 모드 (EE command를 어디서 받을지) 결정

  if (true) Test_trajectory_generator_2dof();
  else if (true) trajectory_gimbaling();
}


void InchControl::Test_trajectory_generator_2dof()
{
  //theta_ref[0] = 0.3 * sin (2 *PI * 0.5 * time_real) + 0.3; // sine wave

  if (time_real < 10) theta_ref[0] = 0;
  else theta_ref[0] = PI/4;
}

void InchControl::trajectory_gimbaling()
{
  ROS_INFO("Happy Gimbaling YYEEEAAAHHH!");
}

void InchControl::Experiment_0623_1Link()
{
  //parameters;
  zeta = 1;
  m = 0.2;
  l = 0.55;
  k = 1000;
  d = 0.02;
  /////////////////

  dt = time_loop;

  w0 = (4 * zeta + sqrt(16 * zeta * zeta - 2)) / dt;
  rho = (2 - w0 * w0 * dt * dt) / (4 * w0 * w0);
  k1 = 2 / (dt * dt + 4 * rho);
  k3 = k1;
  k2 = (2 * dt * dt + 4 * rho) / (dt * dt * dt + 4 * rho * dt);


  k1=1.006;
  k3=k1;
  k2 = 1.8256;
  theta_cmd[0] =  ((1 - (m*l*l) / (12 * k * d * d) * k3) * theta_ref[0] + 
                (m * l * l* k1) / (12 * k * d * d) * q_meas[0] - 
                (m * l * l* k2) / (12 * k * d * d) * q_meas_dot[0] + 
                (m * 9.81 * l) / (8 * k * d * d) * cos(q_meas[0]));

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
}


/*****************************************************************************
** Subscriber callbacks
*****************************************************************************/

void InchControl::dynamixel_workbench_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
  theta_meas[0] = msg->position.at(0);
//  theta_dot_meas[0] = msg->velocity.at(0);

}

void InchControl::Optitrack_callback(const geometry_msgs::Vector3 &msg)
{
  q_meas[0] = msg.x;

if (q_meas[0] - q_meas[1] !=0 ) q_meas_dot[0] = (q_meas[0] - q_meas[1]) / time_loop;
  q_meas_dot[1] = q_meas_dot[0]; // law data
  q_meas_dot[0] = inch_EE_1_misc_->butterworth_2nd_filter(q_meas_dot[0], time_loop);

  q_meas[1] = q_meas[0];
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

  //  inch_ctrl_.SolveInverseForwardKinematics();
    inch_ctrl_.Experiment_0623_1Link();


    //Functions for Control!! - End

    inch_ctrl_.PublishData();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

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
  
  Link1_spring = node_handle_.param<double>("Link1_spring", 0);
  Link2_spring = node_handle_.param<double>("Link2_spring", 0);

  init_X = node_handle_.param<double>("init_X", 0);
  init_Y = node_handle_.param<double>("init_Y", 0);

  Gravity = node_handle_.param<double>("Gravity", 0);

  ROS_INFO("Link1_length[%lf] Link2_length[%lf]", Link1_length, Link2_length);

  N1 = node_handle_.param<double>("N1", 0);
  N2 = node_handle_.param<double>("N2", 0);
  N3 = node_handle_.param<double>("N3", 0);

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
  test_msg.data.resize(20);



  //Init Butterworth 2nd
  inch_q_meas_butterworth->init_butterworth_2nd_filter(40);
  initPoseFlag = true;

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
  theta_command_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/goal_dynamixel_position", 10); // directly to dynamixel
  EE_meas_pub_ = node_handle_.advertise<geometry_msgs::Twist>(robot_name_ + "/EE_meas", 10);
  EE_ref_pub_ = node_handle_.advertise<geometry_msgs::Twist>(robot_name_ + "/EE_ref", 10);

  // Tester publisher
  test_pub_ = node_handle_.advertise<std_msgs::Float64MultiArray>(robot_name_ + "test_Pub", 10);
}

void InchControl::initSubscriber()
{
  gimbal_EE_cmd_sub_ = node_handle_.subscribe("/inch/gimbal_EE_cmd", 10, &InchControl::inch_gimbal_EE_cmd_callback, this, ros::TransportHints().tcpNoDelay());
  
  inch_gimbal_Flag_server_ = node_handle_.advertiseService("/inch/gimbalSrv", &InchControl::gimbal_callback, this);
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
  theta_command_msg.position.push_back(theta_cmd[1]);

  theta_command_pub_.publish(theta_command_msg);


  //inch/EE_meas
  //End Effector position from FK
  EE_meas_msg.linear.y = EE_meas[0];
  EE_meas_msg.linear.z = EE_meas[1];
  EE_meas_pub_.publish(EE_meas_msg);

  //inch/EE_cmd
  //End Effector position ref
  EE_ref_msg.linear.y = EE_ref[0];
  EE_ref_msg.linear.z = EE_ref[1];
  EE_ref_msg.angular.y = EE_cmd[0];
  EE_ref_msg.angular.z = EE_cmd[1];
  EE_ref_pub_.publish(EE_ref_msg);



  //inch/test_Pub
  //Just test
  test_msg.data[0] = theta_cmd[0]; // 
  test_msg.data[1] = inch_joint->q_dot_meas[0]; // Motor command
  test_msg.data[2] = inch_joint->q_meas[0];




  test_msg.data[6] = inch_joint->tau_phi[0]; 
  test_msg.data[7] = inch_joint->tau_MCG[0]; 
  test_msg.data[8] = inch_joint->phi_meas[1]; 
  test_msg.data[9] = inch_joint->phi_MPC[1]; 




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

  if (gimbal_Flag) Test_trajectory_generator_2dof();
  else trajectory_gimbaling();
}

void InchControl::Test_trajectory_generator_2dof()
{
  EE_ref[0] = init_pose[0] + 0.05 * sin (2 *PI * 0.5 * time_real)- 0.07; // sine wave
  EE_ref[1] = init_pose[1];

  // init_pose[0] += 0.05 *sin (2 *PI * 0.3 * time_real);
  // init_pose[1] = init_pose[1];


  // EE_meas_msg.linear.y = EE_meas[0];
  // EE_meas_msg.linear.z = EE_meas[1];

  // EE_ref_msg.linear.y = EE_ref[0];
  // EE_ref_msg.angular.y = EE_cmd[0];

}

void InchControl::trajectory_gimbaling()
{
  EE_ref[0] = EE_gimbal_cmd[0];
  EE_ref[1] = EE_gimbal_cmd[1];
}


void InchControl::inch_gimbal_EE_cmd_callback(const geometry_msgs::Twist& msg)
{
  EE_gimbal_cmd[0] = msg.linear.y;
  EE_gimbal_cmd[1] = msg.linear.z;
}

bool InchControl::gimbal_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
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

Eigen::Vector2d InchControl::F_ext_processing()
{
  Eigen::Vector2d F_ext_;
  tau_ext = inch_joint->tau_phi - inch_joint->tau_MCG;
  tau_ext[0] = tanh_function(tau_ext[0], 1);
  tau_ext[1] = tanh_function(tau_ext[1], 1);
  
  F_ext_ = ForceEstimation(inch_joint->q_meas, tau_ext);
  return F_ext_;
}

void InchControl::init_pose_function()
{
  for (int i = 0; i < 2; i++)
  {
    if((init_theta[i] - theta_cmd[i]) > 0.001) 
    {
      theta_cmd[i] = theta_cmd[i] + 0.0005;
    }
    else if ((init_theta[i] - theta_cmd[i]) < - 0.001) 
    {
      theta_cmd[i] = theta_cmd[i] - 0.0005; // 0.01 * time_loop              
    }
    else
    {
      theta_cmd[i] = init_theta[i];
    }
  }

  if ((abs(init_theta[0] - theta_cmd[0]) < 0.001) &&
      (abs(init_theta[1] - theta_cmd[1]) < 0.001)
     )
  {
    initPoseFlag = false;
    ROS_WARN("Finished to arrive at the initial pose!");
  }

}


void InchControl::Experiment_0623_1Link()
{

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
  inch_link2_PID = new InchMisc(); // link2에 pid 쓸거임

  inch_link1_PID->init_PID_controller(3, 0.00, 0.0, 40);
  inch_link2_PID->init_PID_controller(3, 0.00, 0.0, 40);
  // P:1 D:0.05 

  init_Admittancey(0.5, 3, 3);
  // init_Admittancez(1, 6, 6);


  //init_Admittance(0.1, 0.5, 0.5);
  inch_joint->init_MPC_controller_2Link(8, 1/sqrt(2), 8, 1/sqrt(2));

  init_pose << 0.17, 0.34;
  EE_ref = init_pose;

  // 초기값 튀는거 방지용 입니다.
  init_theta = InverseKinematics_2dof(init_pose);

  ros::spinOnce();


  ros::Rate init_rate(0.3);
  init_rate.sleep();

  ros::spinOnce();

  theta_cmd = inch_joint->theta_meas;
  
}

void InchControl::SeukWhile()
{
  EE_ref = init_pose;
  // Test_trajectory_generator_2dof();

  F_ext = F_ext_processing();

  // EE_cmd[0] = admittanceControly(EE_ref[0], F_ext[0], time_loop);
  // EE_cmd[1] = EE_ref[1];

  q_ref = InverseKinematics_2dof(EE_ref);
  q_des = CommandVelocityLimit(q_ref, 1, time_loop);

  theta_cmd = inch_joint->MPC_controller_2Link(q_des, time_loop);

//   --------------------------------------------------------------
//   // 3-1rd step: PID + admittance
//   EE_ref = init_pose;

//   F_ext = F_ext_processing();

//   EE_cmd[0] = admittanceControly(EE_ref[0], F_ext[0], time_loop);
//   // EE_cmd[1] = admittanceControlz(EE_ref[1], F_ext[1], time_loop);
//   EE_cmd[1] = EE_ref[1]; 


//   q_ref = InverseKinematics_2dof(EE_cmd);
//   q_des = CommandVelocityLimit(q_ref, 0.1, time_loop);

//   theta_cmd[0] = q_des[0] + inch_link1_PID->PID_controller(q_des[0], inch_joint->q_meas[0], time_loop);
//   theta_cmd[1] = q_des[1] + inch_link2_PID->PID_controller(q_des[1], inch_joint->q_meas[1], time_loop);


/////////////////////////////////////////////////////////////////
  EE_meas = ForwardKinematics_2dof(inch_joint->q_meas);

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "InchControl");
  InchControl inch_ctrl_;

//  inch_ctrl_.YujinInit();
  //inch_ctrl_.HanryungInit();
  inch_ctrl_.SeukInit();


  ros::Rate loop_rate(200);

  while (ros::ok())
  {
    inch_ctrl_.TimeCount();

      if (inch_ctrl_.initPoseFlag)
      {
        inch_ctrl_.init_pose_function();
      }
      else
      {
      inch_ctrl_.SeukWhile();
      }
  
  
    inch_ctrl_.PublishData();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

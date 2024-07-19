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

  init_Y = node_handle_.param<double>("init_Y", 0);
  init_Z = node_handle_.param<double>("init_Z", 0);
  init_pose << init_Y, init_Z;

  Gravity = node_handle_.param<double>("Gravity", 0);

  N1 = node_handle_.param<double>("N1", 0);
  N2 = node_handle_.param<double>("N2", 0);
  N3 = node_handle_.param<double>("N3", 0);

  Link1_init_phi = node_handle_.param<double>("Link1_init_phi", 0);
  Link2_init_phi = node_handle_.param<double>("Link2_init_phi", 0);

  phi_init << Link1_init_phi, Link2_init_phi;



  /************************************************************
  ** class init
  ************************************************************/  

  inch_joint = new InchJoint(); 

  inch_butterworth_F_ext_y = new InchMisc(); //qdot에 butterworth 넣을거임
  inch_butterworth_F_ext_z = new InchMisc(); //qdot에 butterworth 넣을거임

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
  inch_butterworth_F_ext_y->init_butterworth_2nd_filter(40);
  inch_butterworth_F_ext_z->init_butterworth_2nd_filter(40);
  initPoseFlag = true;

  gimbal_Flag = false;

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
  EE_cmd_pub_ = node_handle_.advertise<geometry_msgs::Twist>(robot_name_ + "/EE_cmd", 10);
  F_ext_pub_ = node_handle_.advertise<geometry_msgs::Vector3>(robot_name_ + "/F_ext", 10);
  F_ext_raw_pub_ = node_handle_.advertise<geometry_msgs::Vector3>(robot_name_ + "/F_ext_raw", 10);
  q_ref_pub_ = node_handle_.advertise<geometry_msgs::Vector3>(robot_name_ + "/q_ref", 10);
  q_meas_pub_ = node_handle_.advertise<geometry_msgs::Vector3>(robot_name_ + "/q_meas", 10);

  // Tester publisher
  test_pub_ = node_handle_.advertise<std_msgs::Float64MultiArray>(robot_name_ + "test_Pub", 10);
}

void InchControl::initSubscriber()
{
  gimbal_EE_ref_sub_ = node_handle_.subscribe("/inch/gimbal_EE_ref", 10, &InchControl::inch_gimbal_EE_ref_callback, this, ros::TransportHints().tcpNoDelay());
  sbus_sub_ = node_handle_.subscribe("/sbus", 10, &InchControl::sbus_callback, this, ros::TransportHints().tcpNoDelay());

//  inch_gimbal_Flag_server_ = node_handle_.advertiseService("/inch/gimbalSrv", &InchControl::gimbal_callback, this);
}

void InchControl::initServer()
{
  FextY_server = node_handle_.advertiseService("/inch/Fext_Y_srv", &InchControl::FextY_callback, this);
  FextZ_server = node_handle_.advertiseService("/inch/Fext_Z_srv", &InchControl::FextZ_callback, this);

  admittance_server = node_handle_.advertiseService("/inch/admittance_srv", &InchControl::admittance_callback, this);
}

void InchControl::sbus_callback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
  if(msg->data[0] == 0) gimbal_Flag = false;
  else if(msg->data[0] == 1) gimbal_Flag = true;

  if(msg->data[1] == 0) stop_Flag = false;
  else if(msg->data[1] == 1) stop_Flag = true;


  // sbus 신호 0:off, 1:on
  // 채널별로 0번: gimbal
  //        1번: stop
  // 나중에 조종기 토글 골라서 맞춰바꿔야함
}

bool InchControl::FextY_callback(inch_controllers::FextYFilter::Request& req, inch_controllers::FextYFilter::Response& res)
{
  double Y_filter_cof = req.filter_COF;
  tanh_COF_Y = req.tanh_COF;
  deadzone_Y_max = req.deadzone_max;
  deadzone_Y_min = req.deadzone_min;
  inch_butterworth_F_ext_y->init_butterworth_2nd_filter(Y_filter_cof);


  ROS_INFO("FextY Service [%lf]  [%lf]  [%lf]  [%lf]", Y_filter_cof, tanh_COF_Y, deadzone_Y_max, deadzone_Y_min);

  return true;
}

bool InchControl::FextZ_callback(inch_controllers::FextZFilter::Request& req, inch_controllers::FextZFilter::Response& res)
{
  double Z_filter_cof = req.filter_COF;

  tanh_COF_Z = req.tanh_COF;
  deadzone_Z_max = req.deadzone_max;
  deadzone_Z_min = req.deadzone_min;
  inch_butterworth_F_ext_z->init_butterworth_2nd_filter(Z_filter_cof);


  ROS_INFO("FextZ Service [%lf]  [%lf]  [%lf]  [%lf]", Z_filter_cof, tanh_COF_Y, deadzone_Y_max, deadzone_Y_min);
  return true;
}

bool InchControl::admittance_callback(inch_controllers::admittance::Request& req, inch_controllers::admittance::Response& res)
{
  double y_d = req.y_d;
  double y_k = req.y_k;
  double z_d = req.z_d;
  double z_k = req.z_k;
  
  init_CKAdmittancey(y_d, y_k);
  init_CKAdmittancez(z_d, z_k);
  ROS_INFO("Admittance Service [%lf]  [%lf]  [%lf]  [%lf]", y_d, y_k, z_d, z_k);

  return true;
}
// bool InchControl::gimbal_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
// {
//   if (gimbal_Flag)
//   {
//     ROS_INFO("TF not gimbal mode");
//     gimbal_Flag = false;
//   }
//   else
//   {
//     ROS_INFO("TF gimbaling");
//     gimbal_Flag = true;
//   }

//   return true;
// }


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

  //inch/EE_ref
  //End Effector position ref
  EE_ref_msg.linear.y = EE_ref[0];
  EE_ref_msg.linear.z = EE_ref[1];
  EE_ref_pub_.publish(EE_ref_msg);

  //inch/EE_cmd
  //End Effector position cmd (ref w/ admittance)
  EE_cmd_msg.linear.y = EE_cmd[0];
  EE_cmd_msg.linear.z = EE_cmd[1];
  EE_cmd_pub_.publish(EE_cmd_msg);

  //inch/F_ext
  //F_ext
  F_ext_msg.y = F_ext[0];
  F_ext_msg.z = F_ext[1];
  F_ext_pub_.publish(F_ext_msg);


  //inch/F_ext_raw
  //F_ext_raw (w/o filter)
  F_ext_raw_msg.y = F_ext_raw[0];
  F_ext_raw_msg.z = F_ext_raw[1];
  F_ext_raw_pub_.publish(F_ext_raw_msg);


  //inch/q_ref
  q_ref_msg.x = q_ref[0];
  q_ref_msg.y = q_ref[1];
  q_ref_pub_.publish(q_ref_msg);


  //inch/q_meas
  q_meas_msg.x = inch_joint->q_meas[0];
  q_meas_msg.y = inch_joint->q_meas[1];
  q_meas_pub_.publish(q_meas_msg);


  //inch/test_Pub
  //Just test
  test_msg.data[0] = inch_joint->q_meas[0]; // 
  test_msg.data[1] = inch_joint->q_meas[1]; // Motor command
  test_msg.data[2] = theta_cmd[0];
  test_msg.data[3] = theta_cmd[1];

  test_msg.data[4] = q_ref[0];
  test_msg.data[5] = q_ref[1];
  test_msg.data[6] = F_ext[0];
  test_msg.data[7] = F_ext[1];

  test_msg.data[8] = EE_ref[1];
  test_msg.data[9] = EE_meas[1];


  test_pub_.publish(test_msg);
}

void InchControl::deleteToolbox()
{
  delete inch_joint;
  
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

  if (!gimbal_Flag) 
  {
    Test_trajectory_generator_2dof();
    // ROS_INFO("not gimbaling!");
  }
  else 
  {
    trajectory_gimbaling();
    // ROS_WARN("GImballing!!");  
  }
  // ROS_INFO("EE REF!!  [%lf]  [%lf]", EE_ref[0], EE_ref[1]);
}

void InchControl::Test_trajectory_generator_2dof()
{
  EE_ref[0] = init_pose[0]; // sine wave
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
  EE_ref[0] = EE_gimbal_ref[0];
  EE_ref[1] = EE_gimbal_ref[1];
}


void InchControl::inch_gimbal_EE_ref_callback(const geometry_msgs::Twist& msg)
{
  EE_gimbal_ref[0] = msg.linear.y;
  EE_gimbal_ref[1] = msg.linear.z;
}


Eigen::Vector2d InchControl::F_ext_processing()
{
  Eigen::Vector2d F_ext_;
  tau_ext = inch_joint->tau_phi - inch_joint->tau_MCG;
  F_ext_raw = ForceEstimation(inch_joint->q_meas, tau_ext); // 필터링 전 F_ext

  tau_ext[0] = tanh_function(tau_ext[0], tanh_COF_Y);
  tau_ext[1] = tanh_function(tau_ext[1], tanh_COF_Z);

  tau_ext[0] = Dead_Zone_filter(tau_ext[0], deadzone_Y_max, deadzone_Y_min);
  tau_ext[1] = Dead_Zone_filter(tau_ext[1], deadzone_Z_max, deadzone_Z_min);
  
  F_ext_ = ForceEstimation(inch_joint->q_meas, tau_ext);

  F_ext_[0] = inch_butterworth_F_ext_y->butterworth_2nd_filter(F_ext_[0], time_loop);
  F_ext_[1] = inch_butterworth_F_ext_z->butterworth_2nd_filter(F_ext_[1], time_loop);

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
    
    ros::Rate calibration_loop(0.5);
    ros::spinOnce();
    calibration_loop.sleep();
    inch_joint->phi_offset = phi_init - inch_joint->phi_meas;
    // init pose에 도달했을 때 값이 항상 phi_init이 되도록, phi_offset을 설정.
    calibration_loop.sleep();
    
    ROS_WARN("Now, Control loop start!");
  }

}

void InchControl::stop_Function()
{
  theta_cmd = CommandVelocityLimit(init_theta, 0.3, time_loop);
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

  inch_link1_PID->init_PID_controller(0, 0.01, 0.00, 40);
  inch_link2_PID->init_PID_controller(0, 0.01, 0.00, 40);
  // P:1 D:0.05 

  init_Admittancey(1, 2, 4);
  // init_Admittancez(1, 6, 6);


  //init_Admittance(0.1, 0.5, 0.5);
  inch_joint->init_MPC_controller_2Link(8, 1/sqrt(2), 8, 1/sqrt(2));

  init_CKAdmittancey(3, 5);


  init_pose << 0.20, 0.28;
  EE_ref = init_pose;

  // 초기값 튀는거 방지용 입니다.
  init_theta = InverseKinematics_2dof(init_pose);

  ros::spinOnce();
  
  ros::Rate init_rate(0.5);

  init_rate.sleep();
  ros::spinOnce();

  theta_cmd = inch_joint->theta_meas;
  
}

void InchControl::SeukWhile()
{
  Trajectory_mode();

  F_ext = F_ext_processing();

  q_ref = InverseKinematics_2dof(EE_ref);
  q_des = CommandVelocityLimit(q_ref, 1, time_loop);

  theta_cmd = inch_joint->MPC_controller_2Link(q_des, time_loop);
  theta_cmd[0] += inch_link1_PID->PID_controller(q_des[0], inch_joint->q_meas[0], time_loop);
  theta_cmd[1] += inch_link2_PID->PID_controller(q_des[1], inch_joint->q_meas[1], time_loop);


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
    if (inch_ctrl_.stop_Flag)
    {
      inch_ctrl_.stop_Function();
    }
    else
    {
        if (inch_ctrl_.initPoseFlag)
        {
          inch_ctrl_.init_pose_function();
        }
        else
        {
        inch_ctrl_.SeukWhile();
        }
    }
  
    inch_ctrl_.PublishData();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

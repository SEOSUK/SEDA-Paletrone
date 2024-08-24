/*
  Copyright 2018
*/

#include "my_plugin.h"
#include <geometry_msgs/Twist.h>
#include <pluginlib/class_list_macros.h>
#include <QStringList>



namespace rqt_mypkg_cpp
{



MyPlugin::MyPlugin()
    : rqt_gui_cpp::Plugin()
    , widget_(0)
{
    setObjectName("C++PluginT");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui_.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);
////////////////////////////////////////////////////////////////////////////////////////
  ros::start();
  ros::NodeHandle n;

  QObject::connect(ui_.btn_Y_filter, SIGNAL(clicked(bool)), this, SLOT(Fext_Y_Filter_callback(bool))); // Fext
  QObject::connect(ui_.btn_Z_filter, SIGNAL(clicked(bool)), this, SLOT(Fext_Z_Filter_callback(bool))); // Fext
  QObject::connect(ui_.btn_Admittance, SIGNAL(clicked(bool)), this, SLOT(admittance_callback(bool))); // admit
  QObject::connect(ui_.btn_gimbal, SIGNAL(toggled(bool)), this, SLOT(gimbaling_callback(bool))); // gimbal
  QObject::connect(ui_.btn_stop, SIGNAL(toggled(bool)), this, SLOT(manipul_stop_callback(bool))); // stop

  FextY_client = n.serviceClient<rqt_mypkg::FextYFilter>("/inch/Fext_Y_srv"); 
  FextZ_client = n.serviceClient<rqt_mypkg::FextZFilter>("/inch/Fext_Z_srv"); 
  admittance_client = n.serviceClient<rqt_mypkg::admittance>("/inch/admittance_srv"); // 

  timer_callback_ = n.createTimer(ros::Duration(0.1), &MyPlugin::pub_timer_callback, this);  // set sbus Hz
  sbus_pub_ = n.advertise<std_msgs::Int16MultiArray>("/sbus", 10); // sbus~~

  sbus_data.data.resize(3);
  // sbus 신호 0:off, 1:on
  // 채널별로 0번: gimbal
  //        1번: stop
  // 나중에 바꿔야 함
}




void MyPlugin::shutdownPlugin()
{
    // unregister all publishers here
    publisher.shutdown();
}




void MyPlugin::pub_timer_callback(const ros::TimerEvent&)
{
  sbus_pub_.publish(sbus_data);
  ros::spinOnce();

  // sbus 신호 0:off, 1:on
  // 채널별로 0번: gimbal
  //        1번: stop
  // 나중에 바꿔야 함
}



void MyPlugin::Fext_Y_Filter_callback(bool val)
{
  rqt_mypkg::FextYFilter Service;

  Service.request.filter_COF = ui_.edit_Y_cof->text().toDouble();
  Service.request.tanh_COF = ui_.edit_Y_tanh_COF->text().toDouble();
  Service.request.deadzone_max = ui_.edit_Y_deadzone_max->text().toDouble(); 
  Service.request.deadzone_min = ui_.edit_Y_deadzone_min->text().toDouble();


      if(FextY_client.call(Service))
      {
      ui_.btn_Y_filter->setText("launch");
      ui_.btn_Y_filter->setStyleSheet("background-color:none");
      }
      else
      {
      ui_.btn_Y_filter->setText("Failed launch");
      ui_.btn_Y_filter->setStyleSheet("background-color:Red");
      }

  FextY_client.call(Service);
  ros::spinOnce();
}


void MyPlugin::Fext_Z_Filter_callback(bool val)
{
  rqt_mypkg::FextZFilter Service;

  Service.request.filter_COF = ui_.edit_Z_cof->text().toDouble();
  Service.request.tanh_COF = ui_.edit_Z_tanh_COF->text().toDouble();
  Service.request.deadzone_max = ui_.edit_Z_deadzone_max->text().toDouble(); 
  Service.request.deadzone_min = ui_.edit_Z_deadzone_min->text().toDouble();


      if(FextZ_client.call(Service))
      {
      ui_.btn_Z_filter->setText("launch");
      ui_.btn_Z_filter->setStyleSheet("background-color:none");
      }
      else
      {
      ui_.btn_Z_filter->setText("Failed launch");
      ui_.btn_Z_filter->setStyleSheet("background-color:Red");
      }

  FextZ_client.call(Service);
  ros::spinOnce();
}


void MyPlugin::admittance_callback(bool val)
{
  rqt_mypkg::admittance Service;


  Service.request.y_d = ui_.edit_Y_D->text().toDouble();
  Service.request.y_k = ui_.edit_Y_K->text().toDouble();
  Service.request.z_d = ui_.edit_Z_D->text().toDouble(); 
  Service.request.z_k = ui_.edit_Z_K->text().toDouble();


      if(admittance_client.call(Service))
      {
      ui_.btn_Admittance->setText("launch");
      ui_.btn_Admittance->setStyleSheet("background-color:none");
      }
      else
      {
      ui_.btn_Admittance->setText("Failed launch");
      ui_.btn_Admittance->setStyleSheet("background-color:Red");
      }

  admittance_client.call(Service);
  ros::spinOnce();
}



void MyPlugin::gimbaling_callback(bool val)
{
    if(ui_.btn_gimbal->isChecked())
    {
        sbus_data.data[0] = 1;
        ui_.btn_gimbal->setText("gimbaling");
        ui_.btn_gimbal->setStyleSheet("background-color:Green");

      return ;
    }
    else
    {
      sbus_data.data[0] = 0;
      ui_.btn_gimbal->setText("gimbal");
      ui_.btn_gimbal->setStyleSheet("background-color:none");      
    
      return ;
    }
}


void MyPlugin::manipul_stop_callback(bool val)
{
    if(ui_.btn_stop->isChecked())
    {
        sbus_data.data[1] = 1;
        ui_.btn_stop->setText("Stopping!!");
        ui_.btn_stop->setStyleSheet("background-color:Green");

      return ;
    }
    else
    {
      sbus_data.data[1] = 0;
      ui_.btn_stop->setText("Manipulator Stop");
      ui_.btn_stop->setStyleSheet("background-color:none");      
    
      return ;
    }
}













// //------------------------------스페이스바를 누르면 Kill 상태로 보낼 수 있게 만들려고 했는데 일단 실패함 --------------------//
// void MyPlugin::keyPressEvent(QKeyEvent *event)
// {
//   if(event->key() == Qt::Key_Space)
//   { 
//   ui_.btn_Kill->isChecked();
//   }
// }


// //------------------------------아마도 군집 조종하기 위해 사용할 것 같음.... --------------------//
// void MyPlugin::gotogether(bool val)
// {
//   secondturtles::turtlesrv server;

//     if(ui_.chk_t1->isChecked())
//     {
//     server.request.a = ui_.lbl_a->text().toDouble();
//     server.request.b = ui_.lbl_b->text().toDouble(); 
//     servicecaller.call(server);   
//     ros::spinOnce();    
//     }

//     if(ui_.chk_t2->isChecked())
//     {
//     server.request.c = ui_.lbl_c->text().toDouble();
//     server.request.d = ui_.lbl_d->text().toDouble();    
//     secondservicecaller.call(server);   
//     ros::spinOnce();
//     }

// }


}  // namespace rqt_mypkg_cpp




// #define PLUGINLIB_DECLARE_CLASS(pkg, class_name, class_type, base_class_type)
PLUGINLIB_EXPORT_CLASS(rqt_mypkg_cpp::MyPlugin, rqt_gui_cpp::Plugin)



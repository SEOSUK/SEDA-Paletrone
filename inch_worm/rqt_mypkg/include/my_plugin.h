/*
  Copyright 2018
*/
#ifndef RQT_MYPKG_CPP_MY_PLUGIN_H
#define RQT_MYPKG_CPP_MY_PLUGIN_H

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <QWidget>
#include <iostream>
#include <turtlesim/Pose.h>
#include <rqt_mypkg/admittance.h>
#include <rqt_mypkg/FextYFilter.h>
#include <rqt_mypkg/FextZFilter.h>
#include "rqt_mypkg/ui_my_plugin.h"
#include "std_msgs/Int16MultiArray.h"


#include <std_msgs/UInt16.h>

//#include "rqt_mypkg/FAC_HoverService.h"
//#include <QKeyEvent> 
namespace rqt_mypkg_cpp
{

class MyPlugin : public rqt_gui_cpp::Plugin
{    
    Q_OBJECT
public:
    MyPlugin();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();

  std_msgs::Int16MultiArray sbus_data;


private slots:

    void Fext_Y_Filter_callback(bool val); 
    void Fext_Z_Filter_callback(bool val);    
    void admittance_callback(bool val);
    void gimbaling_callback(bool val);  
    void manipul_stop_callback(bool val);  
    void pub_timer_callback(const ros::TimerEvent&);

 //   bool FAC_Hover_Callback(rqt_mypkg::FAC_HoverService::Request &req, rqt_mypkg::FAC_HoverService::Response &res);
 //   void keyPressEvent(QKeyEvent *event); 
    
    private:
    Ui::MyPluginWidget ui_;
    QWidget* widget_;
    ros::Publisher publisher;         //이건 GUI Shutdown 용이라서 건들면 안 됨.

    ros::Publisher sbus_pub_;
    ros::ServiceClient FextY_client;
    ros::ServiceClient FextZ_client;
    ros::ServiceClient admittance_client;
    ros::Timer timer_callback_;
    

    //    ros::ServiceServer HoverServer;
};

}  // namespace rqt_mypkg_cpp

#endif  // RQT_MYPKG_CPP_MY_PLUGIN_H

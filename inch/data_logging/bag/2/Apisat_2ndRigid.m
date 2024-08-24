% timescale 맞추기 귀차나서 변수선언 ㅋㅋㅋ 웃냐
XLIM_min = 0;
XLIM_max = 80;


bag = rosbag("2ndRigid-14-19.bag");

bag_data_log                 =select(bag,'Topic','/inch/data_log');
data_log_time_2Rigid                =timeseries(bag_data_log);
data_log_msgs_2Rigid                =readMessages(bag_data_log);
time_origin_2Rigid                  =data_log_time_2Rigid.Time(1);
data_log_msgs_size_2Rigid           =size(data_log_msgs_2Rigid);


% Palletrone Data 
inchBase_position_2Rigid             =zeros(data_log_msgs_size_2Rigid(1),3); % OpitTrack Info
inchBase_attitude_2Rigid             =zeros(data_log_msgs_size_2Rigid(1),3); % OpitTrack Info

% Inchworm Data
global_EE_ref_2Rigid                 =zeros(data_log_msgs_size_2Rigid(1),2); % OpitTrack Info
global_EE_cmd_2Rigid                 =zeros(data_log_msgs_size_2Rigid(1),2); % OpitTrack Info
global_EE_meas_2Rigid                =zeros(data_log_msgs_size_2Rigid(1),2); % OpitTrack Info
EE_ref_2Rigid                        =zeros(data_log_msgs_size_2Rigid(1),2);
EE_cmd_2Rigid                        =zeros(data_log_msgs_size_2Rigid(1),2);
EE_meas_2Rigid                       =zeros(data_log_msgs_size_2Rigid(1),2);
F_ext_2Rigid                         =zeros(data_log_msgs_size_2Rigid(1),2);
F_ext_raw_2Rigid                     =zeros(data_log_msgs_size_2Rigid(1),2);
q_ref_2Rigid                         =zeros(data_log_msgs_size_2Rigid(1),2);
q_meas_2Rigid                        =zeros(data_log_msgs_size_2Rigid(1),2);
phi_meas_2Rigid                      =zeros(data_log_msgs_size_2Rigid(1),2);
theta_cmd_2Rigid                     =zeros(data_log_msgs_size_2Rigid(1),2);
tau_ext_2Rigid                       =zeros(data_log_msgs_size_2Rigid(1),2);


% Data 뽀개기  
for i=1:data_log_msgs_size_2Rigid(1)
    global_EE_ref_2Rigid(i,1)        =data_log_msgs_2Rigid{i,1}.Data(1);
    global_EE_ref_2Rigid(i,2)        =data_log_msgs_2Rigid{i,1}.Data(2); 
    global_EE_cmd_2Rigid(i,1)        =data_log_msgs_2Rigid{i,1}.Data(3);
    global_EE_cmd_2Rigid(i,2)        =data_log_msgs_2Rigid{i,1}.Data(4);
    global_EE_meas_2Rigid(i,1)       =data_log_msgs_2Rigid{i,1}.Data(5); 
    global_EE_meas_2Rigid(i,2)       =data_log_msgs_2Rigid{i,1}.Data(6);

    EE_ref_2Rigid(i,1)               =data_log_msgs_2Rigid{i,1}.Data(7);
    EE_ref_2Rigid(i,2)               =data_log_msgs_2Rigid{i,1}.Data(8);
    EE_cmd_2Rigid(i,1)               =data_log_msgs_2Rigid{i,1}.Data(9);
    EE_cmd_2Rigid(i,2)               =data_log_msgs_2Rigid{i,1}.Data(10);
    EE_meas_2Rigid(i,1)              =data_log_msgs_2Rigid{i,1}.Data(11);
    EE_meas_2Rigid(i,2)              =data_log_msgs_2Rigid{i,1}.Data(12);
    
    F_ext_2Rigid(i,1)                =data_log_msgs_2Rigid{i,1}.Data(13);
    F_ext_2Rigid(i,2)                =data_log_msgs_2Rigid{i,1}.Data(14);
    F_ext_raw_2Rigid(i,1)            =data_log_msgs_2Rigid{i,1}.Data(15);
    F_ext_raw_2Rigid(i,2)            =data_log_msgs_2Rigid{i,1}.Data(16);
    
    q_ref_2Rigid(i,1)                =data_log_msgs_2Rigid{i,1}.Data(17);
    q_ref_2Rigid(i,2)                =data_log_msgs_2Rigid{i,1}.Data(18);
    q_meas_2Rigid(i,1)               =data_log_msgs_2Rigid{i,1}.Data(19);
    q_meas_2Rigid(i,2)               =data_log_msgs_2Rigid{i,1}.Data(20);

    inchBase_position_2Rigid(i,1)    =data_log_msgs_2Rigid{i,1}.Data(21);
    inchBase_position_2Rigid(i,2)    =data_log_msgs_2Rigid{i,1}.Data(22);
    inchBase_position_2Rigid(i,3)    =data_log_msgs_2Rigid{i,1}.Data(23);
    inchBase_attitude_2Rigid(i,1)    =data_log_msgs_2Rigid{i,1}.Data(24);
    inchBase_attitude_2Rigid(i,2)    =data_log_msgs_2Rigid{i,1}.Data(25);
    inchBase_attitude_2Rigid(i,3)    =data_log_msgs_2Rigid{i,1}.Data(26);

    phi_meas_2Rigid(i,1)               =data_log_msgs_2Rigid{i,1}.Data(27);
    phi_meas_2Rigid(i,2)               =data_log_msgs_2Rigid{i,1}.Data(28);
    theta_cmd_2Rigid(i,1)              =data_log_msgs_2Rigid{i,1}.Data(29);
    theta_cmd_2Rigid(i,2)              =data_log_msgs_2Rigid{i,1}.Data(30);
    tau_ext_2Rigid(i,1)                =data_log_msgs_2Rigid{i,1}.Data(31);
    tau_ext_2Rigid(i,2)                =data_log_msgs_2Rigid{i,1}.Data(32);

end

d1 = designfilt("lowpassiir",FilterOrder=12, ...
    HalfPowerFrequency=0.15,DesignMethod="butter");

inchBase_attitude_2Rigid = filtfilt(d1,inchBase_attitude_2Rigid);


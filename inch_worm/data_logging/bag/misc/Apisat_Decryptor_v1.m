clear; 
close all; 
clc;

% timescale 맞추기 귀차나서 변수선언 ㅋㅋㅋ 웃냐
XLIM_min = 50;
XLIM_max = 70;


bag = rosbag("240727/inch/1st2Rigid-14-14.bag");

bag_data_log                 =select(bag,'Topic','/inch/data_log');
data_log_time                =timeseries(bag_data_log);
data_log_msgs                =readMessages(bag_data_log);
time_origin                  =data_log_time.Time(1);
data_log_msgs_size           =size(data_log_msgs);


% Palletrone Data 
inchBase_position             =zeros(data_log_msgs_size(1),3); % OpitTrack Info
inchBase_attitude             =zeros(data_log_msgs_size(1),3); % OpitTrack Info

% Inchworm Data
global_EE_ref                 =zeros(data_log_msgs_size(1),2); % OpitTrack Info
global_EE_cmd                 =zeros(data_log_msgs_size(1),2); % OpitTrack Info
global_EE_meas                =zeros(data_log_msgs_size(1),2); % OpitTrack Info
EE_ref                        =zeros(data_log_msgs_size(1),2);
EE_cmd                        =zeros(data_log_msgs_size(1),2);
EE_meas                       =zeros(data_log_msgs_size(1),2);
F_ext                         =zeros(data_log_msgs_size(1),2);
F_ext_raw                     =zeros(data_log_msgs_size(1),2);
q_ref                         =zeros(data_log_msgs_size(1),2);
q_meas                        =zeros(data_log_msgs_size(1),2);
phi_meas                      =zeros(data_log_msgs_size(1),2);
theta_cmd                     =zeros(data_log_msgs_size(1),2);
tau_ext                       =zeros(data_log_msgs_size(1),2);


% Data 뽀개기  
for i=1:data_log_msgs_size(1)
    global_EE_ref(i,1)        =data_log_msgs{i,1}.Data(1);
    global_EE_ref(i,2)        =data_log_msgs{i,1}.Data(2); 
    global_EE_cmd(i,1)        =data_log_msgs{i,1}.Data(3);
    global_EE_cmd(i,2)        =data_log_msgs{i,1}.Data(4);
    global_EE_meas(i,1)       =data_log_msgs{i,1}.Data(5); 
    global_EE_meas(i,2)       =data_log_msgs{i,1}.Data(6);

    EE_ref(i,1)               =data_log_msgs{i,1}.Data(7);
    EE_ref(i,2)               =data_log_msgs{i,1}.Data(8);
    EE_cmd(i,1)               =data_log_msgs{i,1}.Data(9);
    EE_cmd(i,2)               =data_log_msgs{i,1}.Data(10);
    EE_meas(i,1)              =data_log_msgs{i,1}.Data(11);
    EE_meas(i,2)              =data_log_msgs{i,1}.Data(12);
    
    F_ext(i,1)                =data_log_msgs{i,1}.Data(13);
    F_ext(i,2)                =data_log_msgs{i,1}.Data(14);
    F_ext_raw(i,1)            =data_log_msgs{i,1}.Data(15);
    F_ext_raw(i,2)            =data_log_msgs{i,1}.Data(16);
    
    q_ref(i,1)                =data_log_msgs{i,1}.Data(17);
    q_ref(i,2)                =data_log_msgs{i,1}.Data(18);
    q_meas(i,1)               =data_log_msgs{i,1}.Data(19);
    q_meas(i,2)               =data_log_msgs{i,1}.Data(20);

    inchBase_position(i,1)    =data_log_msgs{i,1}.Data(21);
    inchBase_position(i,2)    =data_log_msgs{i,1}.Data(22);
    inchBase_position(i,3)    =data_log_msgs{i,1}.Data(23);
    inchBase_attitude(i,1)    =data_log_msgs{i,1}.Data(24);
    inchBase_attitude(i,2)    =data_log_msgs{i,1}.Data(25);
    inchBase_attitude(i,3)    =data_log_msgs{i,1}.Data(26);

    phi_meas(i,1)               =data_log_msgs{i,1}.Data(27);
    phi_meas(i,2)               =data_log_msgs{i,1}.Data(28);
    theta_cmd(i,1)              =data_log_msgs{i,1}.Data(29);
    theta_cmd(i,2)              =data_log_msgs{i,1}.Data(30);
    tau_ext(i,1)                =data_log_msgs{i,1}.Data(31);
    tau_ext(i,2)                =data_log_msgs{i,1}.Data(32);

end


%% ==========================================================
%===================Palletrone Plot=========================
%===========================================================

% Position  
figure('Name','inchBase Position');

subplot(3,1,1)
plot(data_log_time.Time-time_origin,inchBase_position(:,1),'LineWidth',2.0);
hold on
ylabel('$\bf{x}$ \rm\bf{(m)}',Interpreter='latex')
grid
xlim([XLIM_min XLIM_max]);
ylim([-0.45 0.05]);
title('inchBase.x')%,'FontSize',30)

subplot(3,1,2)
plot(data_log_time.Time-time_origin,inchBase_position(:,2),'LineWidth',2.0);
hold on
ylabel('$\bf{y}$ \rm\bf{(m)}',Interpreter='latex')
grid
xlim([XLIM_min XLIM_max]);
ylim([-0.1 0.3]);
title('inchBase.y')%,'FontSize',30)

subplot(3,1,3)
plot(data_log_time.Time-time_origin,inchBase_position(:,3),'LineWidth',2.0);
hold on
ylabel('$\bf{z}$ \rm\bf{(m)}',Interpreter='latex')
grid
xlim([XLIM_min XLIM_max]);
ylim([0.75 1.25]);
title('inchBase.z')%,'FontSize',30)


% Attitude 
figure('Name','inchBase Attitude');

subplot(311)  
plot(data_log_time.Time-time_origin,rad2deg(inchBase_attitude(:,1)),'LineWidth',2.0);
hold on
ylabel('$\phi_F$ \rm\bf{(deg)}',Interpreter='latex')
grid
xlim([XLIM_min XLIM_max]);
ylim([-5 5]);
title('inchBase.roll')%,'FontSize',30)

subplot(312)  
plot(data_log_time.Time-time_origin,rad2deg(inchBase_attitude(:,2)),'LineWidth',2.0);
hold on
ylabel('$\theta_F$ \rm\bf{(deg)}',Interpreter='latex')
grid
xlim([XLIM_min XLIM_max]);
ylim([-5 5]);
title('inchBase.pitch')%,'FontSize',30)

subplot(313)  
plot(data_log_time.Time-time_origin,rad2deg(inchBase_attitude(:,3)),'LineWidth',2.0);
hold on
ylabel('$\psi_F$ \rm\bf{(deg)}',Interpreter='latex')
grid
xlim([XLIM_min XLIM_max]);
ylim([10 20]);
title('inchBase.yaw')%,'FontSize',30)




%% ==========================================================
%===================Inchworm Plot=========================
%===========================================================

%% Global EE : ref, cmd, meas
figure('Name','Global EE');

subplot(211) 
plot(data_log_time.Time-time_origin,global_EE_ref(:,1),'-','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,global_EE_cmd(:,1),'LineWidth',2.0);
plot(data_log_time.Time-time_origin,global_EE_meas(:,1),'LineWidth',2.0);
legend({'$\bf{ref}','$\bf{cmd}','$\bf{meas}'},'Location','northwest','Orientation','horizontal');
% xlabel('$\bf{Time [sec]}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf{y [m]}$',Interpreter='latex')%,'FontSize',24)
grid
xlim([XLIM_min XLIM_max]);
ylim([0.1 0.4]);
title('global_EE.y')%,'FontSize',30)

subplot(212) 
plot(data_log_time.Time-time_origin,global_EE_ref(:,2),'-','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,global_EE_cmd(:,2),'LineWidth',2.0);
plot(data_log_time.Time-time_origin,global_EE_meas(:,2),'LineWidth',2.0);
legend({'$\bf{ref}','$\bf{cmd}','$\bf{meas}'},'Location','northwest','Orientation','horizontal');
xlabel('$\bf{Time [sec]}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf{z [m]}$',Interpreter='latex')%,'FontSize',24)
grid
xlim([XLIM_min XLIM_max]);
ylim([1. 1.4]);
title('global_EE.z')%,'FontSize',30)

%% EE : ref, cmd, meas
figure('Name','Body EE');

subplot(211) 
plot(data_log_time.Time-time_origin,EE_ref(:,1),'-','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,EE_cmd(:,1),'LineWidth',2.0);
plot(data_log_time.Time-time_origin,EE_meas(:,1),'LineWidth',2.0);
legend({'$\bf{ref}','$\bf{cmd}','$\bf{meas}'},'Location','northwest','Orientation','horizontal');
xlabel('$\bf{Time [sec]}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf{y [m]}$',Interpreter='latex')%,'FontSize',24)
grid
xlim([XLIM_min XLIM_max]);
ylim([0.1 0.3]);
title('EE.y')%,'FontSize',30)

subplot(212) 
plot(data_log_time.Time-time_origin,EE_ref(:,2),'-','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,EE_cmd(:,2),'LineWidth',2.0);
plot(data_log_time.Time-time_origin,EE_meas(:,2),'LineWidth',2.0);
legend({'$\bf{ref}','$\bf{cmd}','$\bf{meas}'},'Location','northwest','Orientation','horizontal');
xlabel('$\bf{Time [sec]}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf{z [m]}$',Interpreter='latex')%,'FontSize',24)
grid
xlim([XLIM_min XLIM_max]);
ylim([0. 0.4]);
title('EE.z')%,'FontSize',30)


%% F_ext : Raw, Filtered
figure('Name','External Force');

subplot(211) 
plot(data_log_time.Time-time_origin,F_ext(:,1),'-','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,F_ext_raw(:,1),'LineWidth',2.0);
legend({'$\bf{filtered}','$\bf{raw}'},'Location','northwest','Orientation','horizontal');
xlabel('$\bf{Time [sec]}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf{y [N]}$',Interpreter='latex')%,'FontSize',24)
grid
xlim([XLIM_min XLIM_max]);
ylim([-6 6]);
title('F_{ext}.y')%,'FontSize',30)

subplot(212) 
plot(data_log_time.Time-time_origin,F_ext(:,2),'-','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,F_ext_raw(:,2),'LineWidth',2.0);
legend({'$\bf{filtered}','$\bf{raw}'},'Location','northwest','Orientation','horizontal');
xlabel('$\bf{Time [sec]}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf{z [N]}$',Interpreter='latex')%,'FontSize',24)
grid
xlim([XLIM_min XLIM_max]);
ylim([-6 6]);
title('F_{ext}.z')%,'FontSize',30)


%% q : ref, meas
figure('Name','Link angle');

subplot(211) 
plot(data_log_time.Time-time_origin,rad2deg(q_ref(:,1)),'-','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,rad2deg(q_meas(:,1)),'LineWidth',2.0);
legend({'$\bf{ref}','$\bf{meas}'},'Location','northwest','Orientation','horizontal');
xlabel('$\bf{Time [sec]}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf{q_1 [deg]}$',Interpreter='latex')%,'FontSize',24)
grid
xlim([XLIM_min XLIM_max]);
 ylim([14 34]);
title('q_1')%,'FontSize',30)

subplot(212) 
plot(data_log_time.Time-time_origin,rad2deg(q_ref(:,2)),'-','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,rad2deg(q_meas(:,2)),'LineWidth',2.0);
legend({'$\bf{ref}','$\bf{meas}'},'Location','northwest','Orientation','horizontal');
xlabel('$\bf{Time [sec]}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf{q_2 [deg]}$',Interpreter='latex')%,'FontSize',24)
grid
xlim([XLIM_min XLIM_max]);
 ylim([66 86]);
title('q_2')%,'FontSize',30)


%% phi_meas 
figure('Name','phi_meas');

subplot(211) 
plot(data_log_time.Time-time_origin,rad2deg(phi_meas(:,1)),'LineWidth',2.0);
hold on
xlabel('$\bf{Time [sec]}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf{phi_1 [deg]}$',Interpreter='latex')%,'FontSize',24)
grid
xlim([XLIM_min XLIM_max]);
 ylim([-30 10]);
title('phi_1')%,'FontSize',30)

subplot(212) 
plot(data_log_time.Time-time_origin,rad2deg(phi_meas(:,2)),'LineWidth',2.0);
hold on
xlabel('$\bf{Time [sec]}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf{phi_2 [deg]}$',Interpreter='latex')%,'FontSize',24)
grid
xlim([XLIM_min XLIM_max]);
 ylim([-20 20]);
title('phi_2')%,'FontSize',30)

%% theta_cmd 
figure('Name','theta_cmd');

subplot(211) 
plot(data_log_time.Time-time_origin,rad2deg(theta_cmd(:,1)),'LineWidth',2.0);
hold on
xlabel('$\bf{Time [sec]}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf{theta_1 [deg]}$',Interpreter='latex')%,'FontSize',24)
grid
xlim([XLIM_min XLIM_max]);
 ylim([20 60]);
title('theta_1')%,'FontSize',30)

subplot(212) 
plot(data_log_time.Time-time_origin,rad2deg(theta_cmd(:,2)),'LineWidth',2.0);
hold on
xlabel('$\bf{Time [sec]}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf{theta_2 [deg]}$',Interpreter='latex')%,'FontSize',24)
grid
xlim([XLIM_min XLIM_max]);
 ylim([75 83]);
title('theta_2')%,'FontSize',30)

%% tau_ext
figure('Name','tau_ext');

subplot(211) 
plot(data_log_time.Time-time_origin,rad2deg(tau_ext(:,1)),'LineWidth',2.0);
hold on
xlabel('$\bf{Time [sec]}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf{tau_{ext,2} [Nm]}$',Interpreter='latex')%,'FontSize',24)
grid
xlim([XLIM_min XLIM_max]);
% ylim([7.5 57.5]);
title('tau_ext_1')%,'FontSize',30)

subplot(212) 
plot(data_log_time.Time-time_origin,rad2deg(tau_ext(:,2)),'LineWidth',2.0);
hold on
xlabel('$\bf{Time [sec]}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf{tau_{ext,2} [Nm]}$',Interpreter='latex')%,'FontSize',24)
grid
xlim([XLIM_min XLIM_max]);
% ylim([50 100]);
title('tau_ext_2')%,'FontSize',30)

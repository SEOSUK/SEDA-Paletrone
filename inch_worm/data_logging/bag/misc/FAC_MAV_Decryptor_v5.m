clear all; close all; clc;
bag=rosbag("20240722_1953_paletrone_admitY.bag");
% ssh paletrone@192.168.0.37
% scp paletrone3@192.168.1.10:~/catkin_ws/src/FAC_MAV_paletrone/FAC_MAV/bag/_2023-11-02-22-21-59.bag C:\Users\Admin\Documents\MATLAB
bag_data_log                 =select(bag,'Topic','/data_log');

data_log_time                =timeseries(bag_data_log);

data_log_msgs                =readMessages(bag_data_log);

time_origin                  =data_log_time.Time(1);

data_log_msgs_size           =size(data_log_msgs);

attitude                     =zeros(data_log_msgs_size(1),3);
desired_attitude             =zeros(data_log_msgs_size(1),3);
position                     =zeros(data_log_msgs_size(1),3);
desired_position             =zeros(data_log_msgs_size(1),3);
servo_angle                  =zeros(data_log_msgs_size(1),4);
desired_servo_angle          =zeros(data_log_msgs_size(1),4);
PWM_cmd                      =zeros(data_log_msgs_size(1),8);
desired_torque               =zeros(data_log_msgs_size(1),3);
desired_force                =zeros(data_log_msgs_size(1),3);
individual_motor_thrust      =zeros(data_log_msgs_size(1),4);
sbus                         =zeros(data_log_msgs_size(1),9); 
delta_t                      =zeros(data_log_msgs_size(1),1);
battery_voltage              =zeros(data_log_msgs_size(1),1);
linear_velocity              =zeros(data_log_msgs_size(1),3);
desired_linear_velocity      =zeros(data_log_msgs_size(1),3);
center_of_mass               =zeros(data_log_msgs_size(1),3);
bias_gradient                =zeros(data_log_msgs_size(1),3);
filtered_bias_gradient       =zeros(data_log_msgs_size(1),3);
imu_lin_acceleration         =zeros(data_log_msgs_size(1),3);
external_force               =zeros(data_log_msgs_size(1),3);
admittance_pos_error         =zeros(data_log_msgs_size(1),2);
reference_position           =zeros(data_log_msgs_size(1),3);
angular_velocity             =zeros(data_log_msgs_size(1),3);
attitude_dob_disturbance     =zeros(data_log_msgs_size(1),3);
external_torque              =zeros(data_log_msgs_size(1),3);
mhe_delta_t                  =zeros(data_log_msgs_size(1),1);
calculated_force             =zeros(data_log_msgs_size(1),3);
non_bias_external_force      =zeros(data_log_msgs_size(1),3);
force_dhat                   =zeros(data_log_msgs_size(1),3);
torque_dhat                   =zeros(data_log_msgs_size(1),3);

for i=1:data_log_msgs_size(1)
    attitude(i,1)                     =data_log_msgs{i,1}.Data(1);
    attitude(i,2)                     =data_log_msgs{i,1}.Data(2);
    attitude(i,3)                     =data_log_msgs{i,1}.Data(3);
    desired_attitude(i,1)             =data_log_msgs{i,1}.Data(4);
    desired_attitude(i,2)             =data_log_msgs{i,1}.Data(5);
    desired_attitude(i,3)             =data_log_msgs{i,1}.Data(6);
    position(i,1)                     =data_log_msgs{i,1}.Data(7);
    position(i,2)                     =data_log_msgs{i,1}.Data(8);
    position(i,3)                     =data_log_msgs{i,1}.Data(9);
    desired_position(i,1)             =data_log_msgs{i,1}.Data(10);
    desired_position(i,2)             =data_log_msgs{i,1}.Data(11);
    desired_position(i,3)             =data_log_msgs{i,1}.Data(12);
    servo_angle(i,1)                  =data_log_msgs{i,1}.Data(13);
    servo_angle(i,2)                  =data_log_msgs{i,1}.Data(14);
    servo_angle(i,3)                  =data_log_msgs{i,1}.Data(15);
    servo_angle(i,4)                  =data_log_msgs{i,1}.Data(16);
    desired_servo_angle(i,1)          =data_log_msgs{i,1}.Data(17);
    desired_servo_angle(i,2)          =data_log_msgs{i,1}.Data(18);
    desired_servo_angle(i,3)          =data_log_msgs{i,1}.Data(19);
    desired_servo_angle(i,4)          =data_log_msgs{i,1}.Data(20);
    PWM_cmd(i,1)                      =data_log_msgs{i,1}.Data(21);
    PWM_cmd(i,2)                      =data_log_msgs{i,1}.Data(22);
    PWM_cmd(i,3)                      =data_log_msgs{i,1}.Data(23);
    PWM_cmd(i,4)                      =data_log_msgs{i,1}.Data(24);
    PWM_cmd(i,5)                      =data_log_msgs{i,1}.Data(25);
    PWM_cmd(i,6)                      =data_log_msgs{i,1}.Data(26);
    PWM_cmd(i,7)                      =data_log_msgs{i,1}.Data(27);
    PWM_cmd(i,8)                      =data_log_msgs{i,1}.Data(28);
    desired_torque(i,1)               =data_log_msgs{i,1}.Data(29);
    desired_torque(i,2)               =data_log_msgs{i,1}.Data(30);
    desired_torque(i,3)               =data_log_msgs{i,1}.Data(31);
    desired_force(i,1)                =data_log_msgs{i,1}.Data(32);
    desired_force(i,2)                =data_log_msgs{i,1}.Data(33);
    desired_force(i,3)                =data_log_msgs{i,1}.Data(34);
    individual_motor_thrust(i,1)      =data_log_msgs{i,1}.Data(35);
    individual_motor_thrust(i,2)      =data_log_msgs{i,1}.Data(36);
    individual_motor_thrust(i,3)      =data_log_msgs{i,1}.Data(37);
    individual_motor_thrust(i,4)      =data_log_msgs{i,1}.Data(38);
    sbus(i,1)                         =data_log_msgs{i,1}.Data(39);
    sbus(i,2)                         =data_log_msgs{i,1}.Data(40);
    sbus(i,3)                         =data_log_msgs{i,1}.Data(41);
    sbus(i,4)                         =data_log_msgs{i,1}.Data(42);
    sbus(i,5)                         =data_log_msgs{i,1}.Data(43);
    sbus(i,6)                         =data_log_msgs{i,1}.Data(44);
    sbus(i,7)                         =data_log_msgs{i,1}.Data(45);
    sbus(i,8)                         =data_log_msgs{i,1}.Data(46);
    sbus(i,9)                         =data_log_msgs{i,1}.Data(47);
    delta_t(i,1)                      =data_log_msgs{i,1}.Data(48);
    battery_voltage(i,1)              =data_log_msgs{i,1}.Data(49);
    linear_velocity(i,1)              =data_log_msgs{i,1}.Data(50);
    linear_velocity(i,2)              =data_log_msgs{i,1}.Data(51);
    linear_velocity(i,3)              =data_log_msgs{i,1}.Data(52);
    desired_linear_velocity(i,1)      =data_log_msgs{i,1}.Data(53);
    desired_linear_velocity(i,2)      =data_log_msgs{i,1}.Data(54);
    desired_linear_velocity(i,3)      =data_log_msgs{i,1}.Data(55);
    center_of_mass(i,1)               =data_log_msgs{i,1}.Data(56);
    center_of_mass(i,2)               =data_log_msgs{i,1}.Data(57);
    center_of_mass(i,3)               =data_log_msgs{i,1}.Data(58);
    bias_gradient(i,1)                =data_log_msgs{i,1}.Data(59);
    bias_gradient(i,2)                =data_log_msgs{i,1}.Data(60);
    bias_gradient(i,3)                =data_log_msgs{i,1}.Data(61);
    filtered_bias_gradient(i,1)       =data_log_msgs{i,1}.Data(62);
    filtered_bias_gradient(i,2)       =data_log_msgs{i,1}.Data(63);
    filtered_bias_gradient(i,3)       =data_log_msgs{i,1}.Data(64);
    angular_velocity(i,1)             =data_log_msgs{i,1}.Data(65);
    angular_velocity(i,2)             =data_log_msgs{i,1}.Data(66);
    angular_velocity(i,3)             =data_log_msgs{i,1}.Data(67);
    attitude_dob_disturbance(i,1)     =data_log_msgs{i,1}.Data(68);
    attitude_dob_disturbance(i,2)     =data_log_msgs{i,1}.Data(69);
    attitude_dob_disturbance(i,3)     =data_log_msgs{i,1}.Data(70);
    external_force(i,1)               =data_log_msgs{i,1}.Data(71);
    external_force(i,2)               =data_log_msgs{i,1}.Data(72);
    external_force(i,3)               =data_log_msgs{i,1}.Data(73);
    external_torque(i,1)              =data_log_msgs{i,1}.Data(74);
    external_torque(i,2)              =data_log_msgs{i,1}.Data(75);
    external_torque(i,3)              =data_log_msgs{i,1}.Data(76);
    mhe_delta_t(i,1)                  =data_log_msgs{i,1}.Data(77);
    reference_position(i,1)           =data_log_msgs{i,1}.Data(78);
    reference_position(i,2)           =data_log_msgs{i,1}.Data(79);
    reference_position(i,3)           =data_log_msgs{i,1}.Data(80);
    calculated_force(i,1)             =data_log_msgs{i,1}.Data(81);
    calculated_force(i,2)             =data_log_msgs{i,1}.Data(82);
    calculated_force(i,3)             =data_log_msgs{i,1}.Data(83);
    non_bias_external_force(i,1)      =data_log_msgs{i,1}.Data(84);
    non_bias_external_force(i,2)      =data_log_msgs{i,1}.Data(85);
    non_bias_external_force(i,3)      =data_log_msgs{i,1}.Data(86);
    force_dhat(i,1)                   =data_log_msgs{i,1}.Data(88);
    force_dhat(i,2)                   =data_log_msgs{i,1}.Data(89);
    force_dhat(i,3)                   =data_log_msgs{i,1}.Data(90);
    torque_dhat(i,1)                  =data_log_msgs{i,1}.Data(91);
    torque_dhat(i,2)                  =data_log_msgs{i,1}.Data(92);
    torque_dhat(i,3)                  =data_log_msgs{i,1}.Data(93);
    imu_lin_acceleration(i,1)         =data_log_msgs{i,1}.Data(94);
    imu_lin_acceleration(i,2)         =data_log_msgs{i,1}.Data(95);
    imu_lin_acceleration(i,3)         =data_log_msgs{i,1}.Data(96);
    %admittance_pos_error(i,1)         =data_log_msgs{i,1}.Data(71);
    %admittance_pos_error(i,2)         =data_log_msgs{i,1}.Data(72);
    %reference_position(i,1)           =data_log_msgs{i,1}.Data(73);
    %reference_position(i,2)           =data_log_msgs{i,1}.Data(74);
    %reference_position(i,3)           =data_log_msgs{i,1}.Data(75);
end
% All in one

close all
for a=1
% Attitude


figure('Name','Attitude');
subplot(311)
plot(data_log_time.Time-time_origin,desired_attitude(:,1),'-','LineWidth',2.0);
% plot(servoangle_data.Time-time_origin,-servoangle(:,2),'LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,attitude(:,1),'-','LineWidth',2.0);
legend({'\phi_{F,d}','\phi_F'},'Location','northwest','Orientation','horizontal');
grid
% xlim([240 265]);
ylim([-0.4 0.4]);
subplot(312)
plot(data_log_time.Time-time_origin,desired_attitude(:,2),'-','LineWidth',2.0);
% plot(servoangle_data.Time-time_origin,servoangle(:,1),'LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,attitude(:,2),'-','LineWidth',2.0);
legend({'\theta_{F,d}','\theta_F'},'Location','northwest','Orientation','horizontal');  
grid
% xlim([240 265]);
ylim([-0.4 0.4]);
subplot(313)
plot(data_log_time.Time-time_origin,desired_attitude(:,3),'-','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,attitude(:,3),'-','LineWidth',2.0);
legend({'\psi_{F,d}','\psi_F'},'Location','northwest','Orientation','horizontal');
grid
% xlim([240 265]);
ylim([-pi/2 pi/2]);

%  XYZ Position--------------------------------------------------
figure('Name','Position');
subplot(3,1,1)
plot(data_log_time.Time-time_origin,desired_position(:,1),'-.','LineWidth',2.0);
% plot(lin_vel_data.Time-time_origin,lin_vel(:,1),'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,position(:,1),'LineWidth',2.0);
% set(gca,'FontSize',15);
ylabel('$\bf{x}$ \rm\bf{(m)}',Interpreter='latex')
legend('$^{G}x_d$','$^{G}x$','Interpreter','latex','Orientation','horizontal')%,FontSize=20);
grid
% xlim([240 265]);
ylim([-1. 1.]);
title('Position')%,'FontSize',30)
subplot(3,1,2)
plot(data_log_time.Time-time_origin,desired_position(:,2),'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,position(:,2),'LineWidth',2.0);
% set(gca,'FontSize')%,15);
xlabel('$\bf{Time (sec)}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf{y}$ \rm\bf{(m)}',Interpreter='latex')
legend('$^{G}y_d$','$^{G}y$','Interpreter','latex','Orientation','horizontal')%,FontSize=20);
grid
% xlim([60 85]);
ylim([-1. 1.]);
subplot(3,1,3)
plot(data_log_time.Time-time_origin,desired_position(:,3),'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,position(:,3),'LineWidth',2.0);
% set(gca,'FontSize')%,15);
xlabel('$\bf{Time (sec)}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf{z}$ \rm\bf{(m)}',Interpreter='latex')
legend('$^{G}y_d$','$^{G}y$','Interpreter','latex','Orientation','horizontal')%,FontSize=20);
grid
% xlim([60 85]);
ylim([-2. 1.]);

% Force-------------------------------
figure('Name','Force');
% subplot(511)
% plot(desired_angle_data.Time-time_origin,desired_angle_data.Data(:,3),'-','LineWidth',2.0);
% hold on
% plot(angle_data.Time-time_origin,angle_data.Data(:,3),'-','LineWidth',2.0);
% legend({'\psi_{F,d}','\psi_F'},'Location','northwest','Orientation','horizontal');
% grid
% xlim([55 70]);
% ylim([-3.14 3.14]);
subplot(411)
plot(data_log_time.Time-time_origin,individual_motor_thrust(:,1),'-','LineWidth',2.0);
hold on
title('F_1')
grid
% xlim([55 70]);
% ylim([2 9]);
subplot(412)
plot(data_log_time.Time-time_origin,individual_motor_thrust(:,2),'-','LineWidth',2.0);
hold on
title('F_2')
grid
% xlim([55 70]);
% ylim([2 9]);
subplot(413)
plot(data_log_time.Time-time_origin,individual_motor_thrust(:,3),'-','LineWidth',2.0);
hold on
title('F_3')
grid
% xlim([55 70]);
% ylim([2 7]);
subplot(414)
plot(data_log_time.Time-time_origin,individual_motor_thrust(:,4),'-','LineWidth',2.0);
hold on
title('F_4')
grid
% xlim([55 70]);
% ylim([2 7]);
% sgtitle('Fixed Yaw')

% PWM--------------------------
figure('Name','PWM');
subplot(511)
plot(data_log_time.Time-time_origin,PWM_cmd(:,1),'-','LineWidth',2.0);
grid
% xlim([47 61]);
ylim([1000 2000]);
subplot(512)
plot(data_log_time.Time-time_origin,PWM_cmd(:,2),'-','LineWidth',2.0);
grid
% xlim([47 61]);
ylim([1000 2000]);
subplot(513)
plot(data_log_time.Time-time_origin,PWM_cmd(:,3),'-','LineWidth',2.0);
grid
% xlim([47 61]);
ylim([1000 2000]);
subplot(514)
plot(data_log_time.Time-time_origin,PWM_cmd(:,4),'-','LineWidth',2.0);
grid
% xlim([47 61]);
ylim([1000 2000]);

subplot(515)
grid
plot(data_log_time.Time-time_origin,battery_voltage,'-','LineWidth',2.0);
title('voltage')

% Sbus---------------------------------------------
figure('Name','Sbus');
subplot(421)
plot(data_log_time.Time-time_origin,sbus(:,1),'-','LineWidth',2.0);
hold on
title('Sbus[0],yaw')
% xlim([38 45]);
grid
subplot(423)
plot(data_log_time.Time-time_origin,sbus(:,2),'-','LineWidth',2.0);
hold on
title('Sbus[1],x')
% xlim([38 45]);
grid
subplot(425)
plot(data_log_time.Time-time_origin,sbus(:,3),'-','LineWidth',2.0);
hold on
title('Sbus[2],altitude')
% xlim([38 45]);
grid
subplot(427)
plot(data_log_time.Time-time_origin,sbus(:,4),'-','LineWidth',2.0);
hold on
title('Sbus[3],y')
% xlim([38 45]);
grid

subplot(422)
plot(data_log_time.Time-time_origin,sbus(:,5),'-','LineWidth',2.0);
hold on
title('Sbus[4],kill')
% xlim([38 45]);
grid
subplot(424)
plot(data_log_time.Time-time_origin,sbus(:,6),'-','LineWidth',2.0);
hold on
title('Sbus[5],thr,alti')
% xlim([38 45]);
grid
subplot(426)
plot(data_log_time.Time-time_origin,sbus(:,7),'-','LineWidth',2.0);
hold on
title('Sbus[6],att,vel,pos')
% xlim([38 45]);
grid
subplot(428)
plot(data_log_time.Time-time_origin,sbus(:,8),'-','LineWidth',2.0);
hold on
title('Sbus[7],tilt')
% xlim([38 45]);
grid

% Linear Velocity plot
figure('Name','Linear Velocity');
subplot(311)
plot(data_log_time.Time-time_origin,desired_linear_velocity(:,1),'-','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,linear_velocity(:,1),'-','LineWidth',2.0);
% plot(imu_data.Time-time_origin,y(:,1))
ylabel('$\bf{\dot{x}}$ \rm\bf{(m/s)}',Interpreter='latex')
legend('$^{G}\dot{x_d}$','$^{G}\dot{x}$','Interpreter','latex','Orientation','horizontal')%,FontSize=20);
title('Velocity')%,'FontSize',30)
ylim([-2 2]);
% xlim([60 85]);
grid
subplot(312)
plot(data_log_time.Time-time_origin,desired_linear_velocity(:,2),'-','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,linear_velocity(:,2),'-','LineWidth',2.0);
ylabel('$\bf{\dot{y}}$ \rm\bf{(m/s)}',Interpreter='latex')
legend('$^{G}\dot{y_d}$','$^{G}\dot{y}$','Interpreter','latex','Orientation','horizontal')%,FontSize=20);
ylim([-2 2]);
% xlim([60 85]);
grid
subplot(313)
% plot(desired_lin_vel_data.Time-time_origin,desired_lin_vel(:,3),'-.','LineWidth',2.0);
% hold on
plot(data_log_time.Time-time_origin,linear_velocity(:,3),'-','LineWidth',2.0);
% ylim([-15 15]);
% xlim([30 55]);
grid
% Servo Angle---------------------------------------------------------------
figure('Name','Servo Angle');
subplot(4,1,1)
plot(data_log_time.Time-time_origin,desired_servo_angle(:,1),'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,servo_angle(:,1),'LineWidth',2.0);
% set(gca,'FontSize',15);
ylabel('$\bf\theta_1 (rad)$',Interpreter='latex')%,'FontSize',24)
legend('\theta_{1,d}','\theta_1','Orientation','horizontal')%,FontSize=20);
grid
% xlim([60 85]);
ylim([-.4 .4]);
title('Servo Angle')%,'FontSize',30)
subplot(4,1,2)
plot(data_log_time.Time-time_origin,desired_servo_angle(:,2),'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,servo_angle(:,2),'LineWidth',2.0);
% set(gca,'FontSize',15);
xlabel('$\bf{Time (sec)}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf\theta_2 (rad)$',Interpreter='latex')%,'FontSize',24)
grid
% xlim([60 85]);
ylim([-.4 .4]);
legend('\theta_{2,d}','\theta_2','Orientation','horizontal')%,FontSize=20);
subplot(4,1,3)
plot(data_log_time.Time-time_origin,desired_servo_angle(:,3),'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,servo_angle(:,3),'LineWidth',2.0);
% set(gca,'FontSize',15);
xlabel('$\bf{Time (sec)}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf\theta_3 (rad)$',Interpreter='latex')%,'FontSize',24)
grid
% xlim([60 85]);
ylim([-.4 .4]);
legend('\theta_{3,d}','\theta_3','Orientation','horizontal')%,FontSize=20);
subplot(4,1,4)
plot(data_log_time.Time-time_origin,desired_servo_angle(:,4),'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,servo_angle(:,4),'LineWidth',2.0);
% set(gca,'FontSize',15);
xlabel('$\bf{Time (sec)}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf\theta_4 (rad)$',Interpreter='latex')%,'FontSize',24)
grid
% xlim([60 85]);
ylim([-.4 .4]);
legend('\theta_{4,d}','\theta_4','Orientation','horizontal')%,FontSize=20);


% Desired_value------------------------------
figure('Name','Desired Value');
subplot(321)
plot(data_log_time.Time-time_origin,desired_torque(:,1),'-','LineWidth',2.0);
hold on
grid
% xlim([55 70]);
ylim([-3 3]);
title('\tau_{r,d}')
subplot(323)
plot(data_log_time.Time-time_origin,desired_torque(:,2),'-','LineWidth',2.0);
hold on
grid
% xlim([55 70]);
ylim([-3 3]);
title('\tau_{p,d}')
subplot(325)
plot(data_log_time.Time-time_origin,desired_torque(:,3),'-','LineWidth',2.0);
hold on
grid
% xlim([55 70]);
ylim([-3 3]);
title('\tau_{y,d}')

subplot(322)
plot(data_log_time.Time-time_origin,desired_force(:,1),'-','LineWidth',2.0);
hold on
grid
% xlim([55 70]);
% ylim([-3.14 3.14]);
title('F_{x,d}')
subplot(324)
plot(data_log_time.Time-time_origin,desired_force(:,2),'-','LineWidth',2.0);
hold on
grid
% xlim([55 70]);
% ylim([-3.14 3.14]);
title('F_{y,d}')
subplot(326)
plot(data_log_time.Time-time_origin,desired_force(:,3),'-','LineWidth',2.0);
hold on
grid
% xlim([55 70]);
% ylim([-3.14 3.14]);
title('F_{z,d}')
sgtitle('Published Desired Torque & Force')

% dt-----------------------------------------
figure('Name','dt');
plot(data_log_time.Time-time_origin,delta_t(:,1),'.');
grid
end

%%
figure
subplot(211)
grid
plot

%%
figure
subplot(212)
grid
plot(data_log_time.Time-time_origin,battery_voltage,'-','LineWidth',2.0);
title('voltage')
%% 
figure
subplot(311)
plot(data_log_time.Time-time_origin,angular_velocity(:,1),'-','LineWidth',2.0);
ylabel('$\bf{\dot{\Phi}}$ \rm\bf{(m/s)}',Interpreter='latex')
title('angular velocity')%,'FontSize',30)
ylim([-2 2]);
% xlim([60 85]);
grid
subplot(312)
plot(data_log_time.Time-time_origin,angular_velocity(:,2),'-','LineWidth',2.0);
ylabel('$\bf{\dot{\Theta}}$ \rm\bf{(m/s)}',Interpreter='latex')
ylim([-2 2]);
% xlim([60 85]);
grid
subplot(313)
% plot(desired_lin_vel_data.Time-time_origin,desired_lin_vel(:,3),'-.','LineWidth',2.0);
% hold on
plot(data_log_time.Time-time_origin,angular_velocity(:,3),'-','LineWidth',2.0);
ylabel('$\bf{\dot{\Psi}}$ \rm\bf{(m/s)}',Interpreter='latex')
% ylim([-15 15]);
% xlim([30 55]);
grid

%% FAC_MAV

figure
subplot(2,3,1)
plot(data_log_time.Time-time_origin,desired_attitude(:,1),'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,attitude(:,1),'LineWidth',2.0);
% set(gca,'FontSize')%,15);
ylabel('$\bf\phi_f (rad)$',Interpreter='latex')%,'FontSize',24)
% lgd1=legend({'\phi_{F,d}','\phi_F'},'Orientation','horizontal');%,FontSize=20);
title('Attitude')%,'FontSize',30)
grid
xlim([111 271]);
ylim([-.4 .4]);

subplot(2,3,4)
plot(data_log_time.Time-time_origin,desired_attitude(:,2),'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,attitude(:,2),'LineWidth',2.0);
% set(gca,'FontSize',15);
xlabel('$\bf{Time (sec)}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf\theta_F (rad)$',Interpreter='latex')%,'FontSize',24)
% legend('\theta_{F,d}','\theta_F','Orientation','horizontal')%,FontSize=20);
grid
xlim([111 271]);
ylim([-.4 .4]);

%postion
subplot(2,3,2)
plot(data_log_time.Time-time_origin,desired_position(:,1),'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,position(:,1),'LineWidth',2.0);
% set(gca,'FontSize',15);
ylabel('$\bf{x}$ \rm\bf{(m)}',Interpreter='latex')
% legend('$^{G}x_d$','$^{G}x$','Interpreter','latex','Orientation','horizontal')%,FontSize=20);
grid
xlim([111 271]);
ylim([-0.2 0.2]);
title('Position')%,'FontSize',30)
subplot(2,3,5)
plot(data_log_time.Time-time_origin,desired_position(:,2),'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,position(:,2),'LineWidth',2.0);
% set(gca,'FontSize')%,15);
xlabel('$\bf{Time (sec)}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf{y}$ \rm\bf{(m)}',Interpreter='latex')
% legend('$^{G}y_d$','$^{G}y$','Interpreter','latex','Orientation','horizontal')%,FontSize=20);
grid
xlim([111 271]);
ylim([-1. 1.]);

%servoangle
subplot(2,3,3)
plot(data_log_time.Time-time_origin,desired_servo_angle(:,1),'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,servo_angle(:,1),'LineWidth',2.0);
% set(gca,'FontSize',15);
ylabel('$\bf\theta_1 (rad)$',Interpreter='latex')%,'FontSize',24)
% legend('\theta_{1,d}','\theta_1','Orientation','horizontal')%,FontSize=20);
grid
xlim([111 271]);
ylim([-.4 .4]);
title('Servo Angle')%,'FontSize',30)
subplot(2,3,6)
plot(data_log_time.Time-time_origin,desired_servo_angle(:,2),'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,servo_angle(:,2),'LineWidth',2.0);
% set(gca,'FontSize',15);
xlabel('$\bf{Time (sec)}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf\theta_2 (rad)$',Interpreter='latex')%,'FontSize',24)
% legend('\theta_{2,d}','\theta_2','Orientation','horizontal')%,FontSize=20);
grid
xlim([111 271]);
ylim([-.4 .4]);

f=gcf;
f.Units='inches';
f.OuterPosition = [0.5 0.5 5 5];
exportgraphics(f,'Experiments_servo.png','Resolution',300)
%% PWM
figure('Name','PWM');
subplot(411)
plot(data_log_time.Time-time_origin,PWM_cmd(:,1),'-','LineWidth',2.0);
grid
% xlim([47 61]);
ylim([1000 2000]);
subplot(412)
plot(data_log_time.Time-time_origin,PWM_cmd(:,2),'-','LineWidth',2.0);
grid
% xlim([47 61]);
ylim([1000 2000]);
subplot(413)
plot(data_log_time.Time-time_origin,PWM_cmd(:,3),'-','LineWidth',2.0);
grid
% xlim([47 61]);
ylim([1000 2000]);
subplot(414)
plot(data_log_time.Time-time_origin,PWM_cmd(:,4),'-','LineWidth',2.0);
grid
% xlim([47 61]);
ylim([1000 2000]);
%% Force
figure('Name','Force');
% subplot(511)
% plot(desired_angle_data.Time-time_origin,desired_angle_data.Data(:,3),'-','LineWidth',2.0);
% hold on
% plot(angle_data.Time-time_origin,angle_data.Data(:,3),'-','LineWidth',2.0);
% legend({'\psi_{F,d}','\psi_F'},'Location','northwest','Orientation','horizontal');
% grid
% xlim([55 70]);
% ylim([-3.14 3.14]);
subplot(411)
plot(data_log_time.Time-time_origin,individual_motor_thrust(:,1),'-','LineWidth',2.0);
hold on
title('F_1')
grid
% xlim([55 70]);
% ylim([2 9]);
subplot(412)
plot(data_log_time.Time-time_origin,individual_motor_thrust(:,2),'-','LineWidth',2.0);
hold on
title('F_2')
grid
% xlim([55 70]);
% ylim([2 9]);
subplot(413)
plot(data_log_time.Time-time_origin,individual_motor_thrust(:,3),'-','LineWidth',2.0);
hold on
title('F_3')
grid
% xlim([55 70]);
% ylim([2 7]);
subplot(414)
plot(data_log_time.Time-time_origin,individual_motor_thrust(:,4),'-','LineWidth',2.0);
hold on
title('F_4')
grid
% xlim([55 70]);
% ylim([2 7]);
% sgtitle('Fixed Yaw')
%% 
figure
plot(data_log_time.Time-time_origin,individual_motor_thrust(:,1)+individual_motor_thrust(:,2)+individual_motor_thrust(:,3)+individual_motor_thrust(:,4),'-','LineWidth',2.0);
hold on
title('F_sum')
grid
%% Attitude


figure('Name','Attitude');
subplot(311)
plot(data_log_time.Time-time_origin,desired_attitude(:,1),'-','LineWidth',2.0);
% plot(servoangle_data.Time-time_origin,-servoangle(:,2),'LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,attitude(:,1),'-','LineWidth',2.0);
legend({'\phi_{F,d}','\phi_F'},'Location','northwest','Orientation','horizontal');
grid
% xlim([240 265]);
ylim([-0.2 0.2]);
subplot(312)
plot(data_log_time.Time-time_origin,desired_attitude(:,2),'-','LineWidth',2.0);
% plot(servoangle_data.Time-time_origin,servoangle(:,1),'LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,attitude(:,2),'-','LineWidth',2.0);
legend({'\theta_{F,d}','\theta_F'},'Location','northwest','Orientation','horizontal');  
grid
% xlim([240 265]);
ylim([-0.4 0.4]);
subplot(313)
plot(data_log_time.Time-time_origin,desired_attitude(:,3),'-','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,attitude(:,3),'-','LineWidth',2.0);
legend({'\psi_{F,d}','\psi_F'},'Location','northwest','Orientation','horizontal');
grid
% xlim([240 265]);
ylim([-pi/2 pi/2]);
%%  XYZ Position
figure('Name','Position');
subplot(3,1,1)
plot(data_log_time.Time-time_origin,desired_position(:,1),'-.','LineWidth',2.0);
% plot(lin_vel_data.Time-time_origin,lin_vel(:,1),'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,position(:,1),'LineWidth',2.0);
% set(gca,'FontSize',15);
ylabel('$\bf{x}$ \rm\bf{(m)}',Interpreter='latex')
legend('$^{G}x_d$','$^{G}x$','Interpreter','latex','Orientation','horizontal')%,FontSize=20);
grid
% xlim([240 265]);
ylim([-1. 1.]);
title('Position')%,'FontSize',30)
subplot(3,1,2)
plot(data_log_time.Time-time_origin,desired_position(:,2),'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,position(:,2),'LineWidth',2.0);
% set(gca,'FontSize')%,15);
xlabel('$\bf{Time (sec)}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf{y}$ \rm\bf{(m)}',Interpreter='latex')
legend('$^{G}y_d$','$^{G}y$','Interpreter','latex','Orientation','horizontal')%,FontSize=20);
grid
% xlim([60 85]);
ylim([-1. 1.]);
subplot(3,1,3)
plot(data_log_time.Time-time_origin,desired_position(:,3),'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,position(:,3),'LineWidth',2.0);
% set(gca,'FontSize')%,15);
xlabel('$\bf{Time (sec)}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf{z}$ \rm\bf{(m)}',Interpreter='latex')
legend('$^{G}y_d$','$^{G}y$','Interpreter','latex','Orientation','horizontal')%,FontSize=20);
grid
% xlim([60 85]);
ylim([-2. 1.]);
%% Linear Velocity plot
figure('Name','Velocity');
subplot(311)
plot(data_log_time.Time-time_origin,desired_linear_velocity(:,1),'-','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,linear_velocity(:,1),'-','LineWidth',2.0);
% plot(imu_data.Time-time_origin,y(:,1))
ylabel('$\bf{\dot{x}}$ \rm\bf{(m/s)}',Interpreter='latex')
legend('$^{G}\dot{x_d}$','$^{G}\dot{x}$','Interpreter','latex','Orientation','horizontal')%,FontSize=20);
title('Velocity')%,'FontSize',30)
ylim([-2 2]);
% xlim([60 85]);
grid
subplot(312)
plot(data_log_time.Time-time_origin,desired_linear_velocity(:,2),'-','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,linear_velocity(:,2),'-','LineWidth',2.0);
ylabel('$\bf{\dot{y}}$ \rm\bf{(m/s)}',Interpreter='latex')
legend('$^{G}\dot{y_d}$','$^{G}\dot{y}$','Interpreter','latex','Orientation','horizontal')%,FontSize=20);
ylim([-2 2]);
% xlim([60 85]);
grid
subplot(313)
plot(data_log_time.Time-time_origin,desired_linear_velocity(:,3),'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,linear_velocity(:,3),'-','LineWidth',2.0);
% ylim([-15 15]);
% xlim([30 55]);
grid
%% Servo Angle
figure('Name','Servo');
subplot(4,1,1)
plot(data_log_time.Time-time_origin,desired_servo_angle(:,1),'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,servo_angle(:,1),'LineWidth',2.0);
% set(gca,'FontSize',15);
ylabel('$\bf\theta_1 (rad)$',Interpreter='latex')%,'FontSize',24)
legend('\theta_{1,d}','\theta_1','Orientation','horizontal')%,FontSize=20);
grid
% xlim([60 85]);
ylim([-.4 .4]);
title('Servo Angle')%,'FontSize',30)
subplot(4,1,2)
plot(data_log_time.Time-time_origin,desired_servo_angle(:,2),'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,servo_angle(:,2),'LineWidth',2.0);
% set(gca,'FontSize',15);
xlabel('$\bf{Time (sec)}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf\theta_2 (rad)$',Interpreter='latex')%,'FontSize',24)
grid
% xlim([60 85]);
ylim([-.4 .4]);
legend('\theta_{2,d}','\theta_2','Orientation','horizontal')%,FontSize=20);
subplot(4,1,3)
plot(data_log_time.Time-time_origin,desired_servo_angle(:,3),'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,servo_angle(:,3),'LineWidth',2.0);
% set(gca,'FontSize',15);
xlabel('$\bf{Time (sec)}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf\theta_3 (rad)$',Interpreter='latex')%,'FontSize',24)
grid
% xlim([60 85]);
ylim([-.4 .4]);
legend('\theta_{3,d}','\theta_3','Orientation','horizontal')%,FontSize=20);
subplot(4,1,4)
plot(data_log_time.Time-time_origin,desired_servo_angle(:,4),'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,servo_angle(:,4),'LineWidth',2.0);
% set(gca,'FontSize',15);
xlabel('$\bf{Time (sec)}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf\theta_4 (rad)$',Interpreter='latex')%,'FontSize',24)
grid
% xlim([60 85]);
ylim([-.4 .4]);
legend('\theta_{4,d}','\theta_4','Orientation','horizontal')%,FontSize=20);
%% IMU Acceleration plot
figure('Name','IMU Accel');
subplot(311)
plot(data_log_time.Time-time_origin,imu_lin_acceleration(:,1),'-','LineWidth',2.0);
hold on
ylabel('$\bf{\ddot{x}} \rm\bf{(m/s^2)}$',Interpreter='latex')
title('IMU acceleration')%,'FontSize',30)
%ylim([-2 2]);
% xlim([60 85]);
grid
subplot(312)
plot(data_log_time.Time-time_origin,imu_lin_acceleration(:,2),'-','LineWidth',2.0);
hold on
ylabel('$\bf{\ddot{y}} \rm\bf{(m/s^2)}$',Interpreter='latex')
%ylim([-2 2]);
% xlim([60 85]);
grid
subplot(313)
plot(data_log_time.Time-time_origin,imu_lin_acceleration(:,3),'-','LineWidth',2.0);
hold on
ylabel('$\bf{\ddot{z}} \rm\bf{(m/s^2)}$',Interpreter='latex')
% ylim([-15 15]);
% xlim([30 55]);
grid

%% Desired_value
figure('Name','Desired');
subplot(321)
plot(data_log_time.Time-time_origin,desired_torque(:,1),'-','LineWidth',2.0);
hold on
grid
% xlim([55 70]);
ylim([-3 3]);
title('\tau_{r,d}')
subplot(323)
plot(data_log_time.Time-time_origin,desired_torque(:,2),'-','LineWidth',2.0);
hold on
grid
% xlim([55 70]);
ylim([-3 3]);
title('\tau_{p,d}')
subplot(325)
plot(data_log_time.Time-time_origin,desired_torque(:,3),'-','LineWidth',2.0);
hold on
grid
% xlim([55 70]);
ylim([-3 3]);
title('\tau_{y,d}')

subplot(322)
plot(data_log_time.Time-time_origin,desired_force(:,1),'-','LineWidth',2.0);
hold on
grid
% xlim([55 70]);
% ylim([-3.14 3.14]);
title('F_{x,d}')
subplot(324)
plot(data_log_time.Time-time_origin,desired_force(:,2),'-','LineWidth',2.0);
hold on
grid
% xlim([55 70]);
% ylim([-3.14 3.14]);
title('F_{y,d}')
subplot(326)
plot(data_log_time.Time-time_origin,desired_force(:,3),'-','LineWidth',2.0);
hold on
grid
% xlim([55 70]);
% ylim([-3.14 3.14]);
title('F_{z,d}')
sgtitle('Published Desired Torque & Force')

%% Sbus
figure
subplot(421)
plot(data_log_time.Time-time_origin,sbus(:,1),'-','LineWidth',2.0);
hold on
title('Sbus[0]')
% xlim([38 45]);
grid
subplot(423)
plot(data_log_time.Time-time_origin,sbus(:,2),'-','LineWidth',2.0);
hold on
title('Sbus[1]')
% xlim([38 45]);
grid
subplot(425)
plot(data_log_time.Time-time_origin,sbus(:,3),'-','LineWidth',2.0);
hold on
title('Sbus[2]')
% xlim([38 45]);
grid
subplot(427)
plot(data_log_time.Time-time_origin,sbus(:,4),'-','LineWidth',2.0);
hold on
title('Sbus[3]')
% xlim([38 45]);
grid

subplot(422)
plot(data_log_time.Time-time_origin,sbus(:,5),'-','LineWidth',2.0);
hold on
title('Sbus[4]')
% xlim([38 45]);
grid
subplot(424)
plot(data_log_time.Time-time_origin,sbus(:,6),'-','LineWidth',2.0);
hold on
title('Sbus[5]')
% xlim([38 45]);
grid
subplot(426)
plot(data_log_time.Time-time_origin,sbus(:,7),'-','LineWidth',2.0);
hold on
title('Sbus[6]')
% xlim([38 45]);
grid
subplot(428)
plot(data_log_time.Time-time_origin,sbus(:,8),'-','LineWidth',2.0);
hold on
title('Sbus[7]')
% xlim([38 45]);
grid
%% dt
figure
plot(data_log_time.Time-time_origin,delta_t(:,1),'.');
grid
%% figure
subplot(2,1,1)
plot(data_log_time.Time-time_origin,desired_force(:,2)*0.1,'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,servo_angle(:,1),'LineWidth',2.0);
% set(gca,'FontSize',15);
ylabel('$\bf\theta_1 (rad)$',Interpreter='latex')%,'FontSize',24)
legend('\theta_{1,d}','\theta_1','Orientation','horizontal')%,FontSize=20);
grid
% xlim([60 85]);
% ylim([-.4 .4]);
title('Servo Angle')%,'FontSize',30)
subplot(2,1,2)
plot(data_log_time.Time-time_origin,desired_force(:,1)*0.1,'-.','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,servo_angle(:,2),'LineWidth',2.0);
% set(gca,'FontSize',15);
xlabel('$\bf{Time (sec)}$',Interpreter='latex')%,'FontSize',24)
ylabel('$\bf\theta_2 (rad)$',Interpreter='latex')%,'FontSize',24)
grid
% xlim([60 85]);
% ylim([-.4 .4]);
legend('\theta_{2,d}','\theta_2','Orientation','horizontal')%,FontSize=20);
%% External Force
figure
subplot(311)
plot(data_log_time.Time-time_origin,external_force(:,1),'-','LineWidth',2.0);
hold on
ylabel('$\bf{X} \rm\bf{(N)}$',Interpreter='latex')
title('External force')%,'FontSize',30)
ylim([-15 15]);
% xlim([60 85]);
grid
subplot(312)
plot(data_log_time.Time-time_origin,external_force(:,2),'-','LineWidth',2.0);
hold on
ylabel('$\bf{Y} \rm\bf{(N)}$',Interpreter='latex')
ylim([-15 15]);
% xlim([60 85]);
grid
subplot(313)
plot(data_log_time.Time-time_origin,external_force(:,3),'-','LineWidth',2.0);
hold on
ylabel('$\bf{Z} \rm\bf{(N)}$',Interpreter='latex')
ylim([-15 15]);
% xlim([30 55]);
grid
%% Admittance result
figure
subplot(211)
plot(data_log_time.Time-time_origin,admittance_pos_error(:,1),'-','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,position(:,1),'-','LineWidth',2.0);
ylabel('$\bf{X} \rm\bf{(m)}$',Interpreter='latex')
title('Admittance result')%,'FontSize',30)
ylim([-2 2]);
% xlim([60 85]);
grid
subplot(212)
plot(data_log_time.Time-time_origin,admittance_pos_error(:,2),'-','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,position(:,2),'-','LineWidth',2.0);
ylabel('$\bf{Y} \rm\bf{(m)}$',Interpreter='latex')
ylim([-2 2]);
% xlim([60 85]);
grid
% subplot(313)
% plot(data_log_time.Time-time_origin,admittance_pos_error(:,3),'-','LineWidth',2.0);
% hold on
% ylabel('$\bf{Z} \rm\bf{(m)}$',Interpreter='latex')
% % ylim([-15 15]);
% xlim([30 55]);
%% Reference postion
figure
subplot(211)
plot(data_log_time.Time-time_origin,reference_position(:,1),'-','LineWidth',2.0);
hold on
% plot(data_log_time.Time-time_origin,desired_position(:,1),'-','LineWidth',2.0);
plot(data_log_time.Time-time_origin,position(:,1),'-','LineWidth',2.0);
ylabel('$\bf{X} \rm\bf{(m)}$',Interpreter='latex')
legend
title('Reference postion')%,'FontSize',30)
ylim([-2 2]);
% xlim([60 85]);
grid
subplot(212)
plot(data_log_time.Time-time_origin,reference_position(:,2),'-','LineWidth',2.0);
hold on
% plot(data_log_time.Time-time_origin,desired_position(:,2),'-','LineWidth',2.0);
plot(data_log_time.Time-time_origin,position(:,2),'-','LineWidth',2.0);
ylabel('$\bf{Y} \rm\bf{(m)}$',Interpreter='latex')
ylim([-2 2]);
legend
% xlim([60 85]);
grid
%% 
d1 = designfilt('lowpassiir','FilterOrder',12, ...
    'HalfPowerFrequency',0.01,'DesignMethod','butter');

m1_filtfilt = filtfilt(d1,individual_motor_thrust(:,1));
m2_filtfilt = filtfilt(d1,individual_motor_thrust(:,2));
m3_filtfilt = filtfilt(d1,individual_motor_thrust(:,3));
m4_filtfilt = filtfilt(d1,individual_motor_thrust(:,4));

figure
plot(data_log_time.Time-time_origin,individual_motor_thrust(:,1));
hold on
plot(data_log_time.Time-time_origin,m1_filtfilt);
grid
%% 
for i=1:data_log_msgs_size(1)
%     SA=[individual_motor_thrust(i,1)/sqrt(2)  individual_motor_thrust(i,2)/sqrt(2) -individual_motor_thrust(i,3)/sqrt(2) -individual_motor_thrust(i,4)/sqrt(2);
%            individual_motor_thrust(i,1)/sqrt(2) -individual_motor_thrust(i,2)/sqrt(2) -individual_motor_thrust(i,3)/sqrt(2)  individual_motor_thrust(i,4)/sqrt(2);
%            0.3025*individual_motor_thrust(i,1) 0.3025*individual_motor_thrust(i,2) 0.3025*individual_motor_thrust(i,3) 0.3025*individual_motor_thrust(i,4)];
    SA=[m1_filtfilt(i)/sqrt(2)  m2_filtfilt(i)/sqrt(2) -m3_filtfilt(i)/sqrt(2) -m4_filtfilt(i)/sqrt(2);
           m1_filtfilt(i)/sqrt(2) -m2_filtfilt(i)/sqrt(2) -m3_filtfilt(i)/sqrt(2)  m4_filtfilt(i)/sqrt(2);
           0.3025*m1_filtfilt(i) 0.3025*m2_filtfilt(i) 0.3025*m3_filtfilt(i) 0.3025*m4_filtfilt(i)];

    desired_sin_theta(i,1:4)=pinv(SA)*[desired_force(i,1) desired_force(i,2) desired_torque(i,3)-(-m1_filtfilt(i)*0.01*cos(servo_angle(i,1))+m2_filtfilt(i)*0.01*cos(servo_angle(i,2))-m3_filtfilt(i)*0.01*cos(servo_angle(i,3))+m4_filtfilt(i)*0.01*cos(servo_angle(i,4)))]';
%     desired_sin_theta(i,1:4)=pinv(SA)*[desired_force(i,1) desired_force(i,2) desired_torque(i,3)-(-individual_motor_thrust(i,1)*0.01*cos(servo_angle(i,1))+individual_motor_thrust(i,2)*0.01*cos(servo_angle(i,2))-individual_motor_thrust(i,3)*0.01*cos(servo_angle(i,3))+individual_motor_thrust(i,4)*0.01*cos(servo_angle(i,4)))]';
end

figure
subplot(411)
plot(data_log_time.Time-time_origin,desired_sin_theta(:,1));
hold on
grid
subplot(412)
plot(data_log_time.Time-time_origin,desired_sin_theta(:,2));
hold on
grid
subplot(413)
plot(data_log_time.Time-time_origin,desired_sin_theta(:,3));
hold on
grid
subplot(414)
plot(data_log_time.Time-time_origin,desired_sin_theta(:,4));
hold on
grid

%%
pppp=sin(desired_servo_angle(:,1)).*individual_motor_thrust(:,1)+sin(desired_servo_angle(:,2)).*individual_motor_thrust(:,2)+sin(desired_servo_angle(:,3)).*individual_motor_thrust(:,3)+sin(desired_servo_angle(:,4)).*individual_motor_thrust(:,4);
pppp=sin(desired_servo_angle(:,1)).*individual_motor_thrust(:,1)-sin(desired_servo_angle(:,2)).*individual_motor_thrust(:,2)+sin(desired_servo_angle(:,3)).*individual_motor_thrust(:,3)-sin(desired_servo_angle(:,4)).*individual_motor_thrust(:,4);
plot(data_log_time.Time-time_origin,pppp);
%%
force3torque(:,1) = desired_force(:,1);
force3torque(:,2) = desired_force(:,2);
force3torque(:,3) = 0;
force3torque(:,4) = 0;
r2= sqrt(2);
r=0.3025;
for i=1:data_log_msgs_size(1)
    A = [r2/individual_motor_thrust(i,1), r2/individual_motor_thrust(i,1), 1/(r*individual_motor_thrust(i,1)), 1/1/(r*individual_motor_thrust(i,1))
        r2/individual_motor_thrust(i,2), -r2/individual_motor_thrust(i,2), 1/(r*individual_motor_thrust(i,2)), -1/1/(r*individual_motor_thrust(i,2))
        -r2/individual_motor_thrust(i,3), -r2/individual_motor_thrust(i,3), 1/(r*individual_motor_thrust(i,3)), 1/1/(r*individual_motor_thrust(i,3))
        -r2/individual_motor_thrust(i,4), r2/individual_motor_thrust(i,4), 1/(r*individual_motor_thrust(i,4)), -1/1/(r*individual_motor_thrust(i,4))];
    sins(i,:) = A * force3torque(i,:)';
end
figure
subplot(411)
plot(data_log_time.Time-time_origin,sins(:,1));
%ylim([-1,1])
grid
subplot(412)
plot(data_log_time.Time-time_origin,sins(:,2));
%ylim([-1,1])
grid
subplot(413)
plot(data_log_time.Time-time_origin,sins(:,3));
%ylim([-1,1])
grid
subplot(414)
plot(data_log_time.Time-time_origin,sins(:,4));
%ylim([-1,1])
grid

%%
syms F1 F2 F3 F4 r
r2=sqrt(2)
A=[F1/r2 F2/r2 -F3/r2 -F4/r2
    F1/r2 -F2/r2 -F3/r2 F4/r2
    F1*r2 F2*r2 F3*r2 F4*r2
    F1*r2 -F2*r2 F3*r2 -F4*r2
    ]
%%
subplot(211)
plot(data_log_time.Time-time_origin,desired_attitude(:,3),'-','LineWidth',2.0);
hold on
plot(data_log_time.Time-time_origin,attitude(:,3),'-','LineWidth',2.0);
legend({'\psi_{F,d}','\psi_F'},'Location','northwest','Orientation','horizontal');
grid
% xlim([240 265]);
ylim([-pi/2 pi/2]);
subplot(212)
plot(data_log_time.Time-time_origin,desired_torque(:,3),'-','LineWidth',2.0);
hold on
grid
% xlim([55 70]);
ylim([-3 3]);
title('\tau_{y,d}')
%% 
figure
plot(data_log_time.Time-time_origin,mhe_delta_t(:,1),'.','LineWidth',2.0);
%% Force DOB
figure('Name','Force DOB');
subplot(311)
plot(data_log_time.Time-time_origin,force_dhat(:,1),'-','LineWidth',2.0);
hold on
ylabel('$\bf{X} \rm\bf{(N)}$',Interpreter='latex')
title('dhat')%,'FontSize',30)
%ylim([-5 5]);
% xlim([60 85]);
grid
subplot(312)
plot(data_log_time.Time-time_origin,force_dhat(:,2),'-','LineWidth',2.0);
hold on
ylabel('$\bf{Y} \rm\bf{(N)}$',Interpreter='latex')
%ylim([-5 5]);
% xlim([60 85]);
grid
subplot(313)
plot(data_log_time.Time-time_origin,force_dhat(:,3),'-','LineWidth',2.0);
hold on
ylabel('$\bf{Z} \rm\bf{(N)}$',Interpreter='latex')
% ylim([-15 15]);
% xlim([30 55]);
grid


%% Torque DOB
figure('Name','Torque DOB');
subplot(311)
plot(data_log_time.Time-time_origin,torque_dhat(:,1),'-','LineWidth',2.0);
hold on
ylabel('$\bf{X} \rm\bf{(N)}$',Interpreter='latex')
title('dhat')%,'FontSize',30)
%ylim([-1 1]);
% xlim([60 85]);
grid
subplot(312)
plot(data_log_time.Time-time_origin,torque_dhat(:,2),'-','LineWidth',2.0);
hold on
ylabel('$\bf{Y} \rm\bf{(N)}$',Interpreter='latex')
%ylim([-1 1]);
% xlim([60 85]);
grid
subplot(313)
plot(data_log_time.Time-time_origin,torque_dhat(:,3),'-','LineWidth',2.0);
hold on
ylabel('$\bf{Z} \rm\bf{(N)}$',Interpreter='latex')
% ylim([-15 15]);
% xlim([30 55]);
grid
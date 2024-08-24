figure('Name','Comparison of Rigid X SEDA X SEDA+Active Admittance Control');
fontname("Times New Roman")

set(gcf,'Color','w')

subplot(231) 
title('SEDA Manipulator')%,'FontSize',30)
hold on
grid on
plot(data_log_time_2Rigid.Time-time_origin_2Rigid,EE_ref_2Rigid(:,2) - 0.28,'k--', 'LineWidth', 2, 'DisplayName', 'EE_{ref}');
plot(data_log_time_2Rigid.Time-time_origin_2Rigid,EE_meas_2Rigid(:,2) - 0.28, 'r-', 'LineWidth', 1.5, 'DisplayName', 'EE_{meas}');
legend show;
legend('Orientation','horizontal')
xlabel('Time [s]', 'FontName', 'Times New Roman');
ylabel('Position Z Error[rad]', 'FontName', 'Times New Roman');
grid on
xlim([74 79]);
ylim([-0.1 0.1]);
title('Rigid')%,'FontSize',30)
fontname("Times New Roman")

subplot(234) 
plot(data_log_time_2Rigid.Time-time_origin_2Rigid,inchBase_attitude_2Rigid(:,1),'k-', 'LineWidth', 1.5, 'DisplayName', '\phi');
hold on
grid on
xlabel('Time [s]', 'FontName', 'Times New Roman');
ylabel('Base Attitude Error[rad]', 'FontName', 'Times New Roman');
legend show;
legend('Orientation','horizontal')
xlim([74 79]);
ylim([-0.2 0.2]);
fontname("Times New Roman")




subplot(232) 
title('SEDA Manipulator')%,'FontSize',30)
hold on
grid on
plot(data_log_time_2SEDA.Time-time_origin_2SEDA,EE_ref_2SEDA(:,2) - 0.35,'k--', 'LineWidth', 2, 'DisplayName', 'EE_{ref}');
plot(data_log_time_2SEDA.Time-time_origin_2SEDA,EE_meas_2SEDA(:,2) - 0.35, 'r-', 'LineWidth', 1.5, 'DisplayName', 'EE_{meas}');
legend show;
legend('Orientation','horizontal')
xlabel('Time [s]', 'FontName', 'Times New Roman');
grid on
xlim([39 44]);
ylim([-0.1 0.1]);
title('SEDA')%,'FontSize',30)
fontname("Times New Roman")

subplot(235) 
plot(data_log_time_2SEDA.Time-time_origin_2SEDA,inchBase_attitude_2SEDA(:,1),'k-', 'LineWidth', 1.5, 'DisplayName', '\phi');
hold on
grid on
xlabel('Time [s]', 'FontName', 'Times New Roman');
legend show;
legend('Orientation','horizontal')
xlim([39 44]);
ylim([-0.2 0.2]);
fontname("Times New Roman")



subplot(233) 
title('SEDA Manipulator')%,'FontSize',30)
hold on
grid on
plot(data_log_time_2ACTIVE.Time-time_origin_2ACTIVE,EE_ref_2ACTIVE(:,2) - 0.35,'k--', 'LineWidth', 2, 'DisplayName', 'EE_{ref}');
plot(data_log_time_2ACTIVE.Time-time_origin_2ACTIVE,EE_cmd_2ACTIVE(:,2) - 0.35,'b-', 'LineWidth', 1.5, 'DisplayName', 'EE_{cmd}');
plot(data_log_time_2ACTIVE.Time-time_origin_2ACTIVE,EE_meas_2ACTIVE(:,2) - 0.35, 'r-', 'LineWidth', 2, 'DisplayName', 'EE_{meas}');
legend show;
legend('Orientation','horizontal')
xlabel('Time [s]', 'FontName', 'Times New Roman');
grid on
xlim([70 75]);
ylim([-0.1 0.1]);
title('SEDA+Active')%,'FontSize',30)
fontname("Times New Roman")

subplot(236) 
plot(data_log_time_2ACTIVE.Time-time_origin_2ACTIVE,inchBase_attitude_2ACTIVE(:,1),'k-', 'LineWidth', 1.5, 'DisplayName', '\phi');
hold on
grid on
legend show;
legend('Orientation','horizontal')
xlabel('Time [s]', 'FontName', 'Times New Roman');
xlim([70 75]);
ylim([-0.2 0.2]);
fontname("Times New Roman")
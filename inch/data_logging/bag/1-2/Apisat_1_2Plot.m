figure('Color', 'w'); 


subplot(1,2,1)
title('SEDA Manipulator')%,'FontSize',30)
hold on
grid on
plot(data_log_time_1_2SEDA.Time-time_origin_1_2SEDA - 60,inchBase_position_1_2SEDA(:,3),'b-', 'LineWidth', 2, 'DisplayName', 'Base Altitude');
plot(data_log_time_1_2SEDA.Time-time_origin_1_2SEDA - 60,global_EE_ref_1_2SEDA(:,2),'k-', 'LineWidth', 2, 'DisplayName', 'EE_{cmd}');
plot(data_log_time_1_2SEDA.Time-time_origin_1_2SEDA - 60,global_EE_meas_1_2SEDA(:,2),'r--', 'LineWidth', 1.5, 'DisplayName', 'EE_{meas}');
legend show;
legend('Orientation','horizontal')
xlabel('Time [s]', 'FontName', 'Times New Roman');
ylabel('Position[m]', 'FontName', 'Times New Roman');
xlim([0 20]);
ylim([0.9 1.5]);

subplot(1,2,2) 
title('Rigid Manipulator');
hold on
grid on
plot(data_log_time_1_2Rigid.Time-time_origin_1_2Rigid - 56,inchBase_position_1_2Rigid(:,3),'b-', 'LineWidth', 2, 'DisplayName', 'Base Altitude');
plot(data_log_time_1_2Rigid.Time-time_origin_1_2Rigid - 56,global_EE_cmd_1_2Rigid(:,2),'k-', 'LineWidth', 2, 'DisplayName', 'EE_{cmd}');
plot(data_log_time_1_2Rigid.Time-time_origin_1_2Rigid - 56,global_EE_meas_1_2Rigid(:,2),'r--', 'LineWidth', 1.5, 'DisplayName', 'EE_{meas}');
legend show;
legend('Orientation','horizontal')
xlabel('Time [s]', 'FontName', 'Times New Roman');
ylabel('Position[m]', 'FontName', 'Times New Roman');
xlim([0 20]);
ylim([0.9 1.5]);


RMSE_SEDA = sqrt(mean((global_EE_ref_1_2SEDA(6200:7000,2) - global_EE_meas_1_2SEDA(6200:7000,2)).^2))
RMSE_RIGID = sqrt(mean((global_EE_cmd_1_2Rigid(5800:6600,2) - global_EE_meas_1_2Rigid(5800:6600,2)).^2))

MAE_SEDA = mean(abs(global_EE_ref_1_2SEDA(6200:7000,2) - global_EE_meas_1_2SEDA(6200:7000,2)))
RMSE_RIGID = mean(abs(global_EE_cmd_1_2Rigid(5800:6600,2) - global_EE_meas_1_2Rigid(5800:6600,2)))
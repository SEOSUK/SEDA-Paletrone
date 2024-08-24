figure('Color', 'w'); 
sgtitle('SEA vs. SEDA')


subplot(2,2,1)
title('q1 error', 'FontName', 'Times New Roman')%,'FontSize',30)
hold on
grid on
plot(data_log_time_1_1SEA.Time-time_origin_1_1SEA - 56, phi_meas_1_1SEA(:, 1)+theta_cmd_1_1SEA(:,1)   ,'r-', 'LineWidth', 1.5, 'DisplayName', 'SEA');
plot(data_log_time_1_1SEDA.Time-time_origin_1_1SEDA - 47 - 0.141, phi_meas_1_1SEDA(:, 1)+theta_cmd_1_1SEDA(:,1) - 0.6,'b', 'LineWidth', 1.5, 'DisplayName', 'SEDA');
xlabel('Time [s]', 'FontName', 'Times New Roman');
ylabel('error[rad]', 'FontName', 'Times New Roman');
xlim([0 10]);
legend show; 
legend('Orientation','horizontal')


subplot(2,2,3)
title('q2 error', 'FontName', 'Times New Roman')%,'FontSize',30)
hold on
grid on
plot(data_log_time_1_1SEA.Time-time_origin_1_1SEA - 56, phi_meas_1_1SEA(:, 2)+theta_cmd_1_1SEA(:,2),'r-', 'LineWidth', 1.5, 'DisplayName', 'SEA');
plot(data_log_time_1_1SEDA.Time-time_origin_1_1SEDA - 47 - 0.141, phi_meas_1_1SEDA(:, 2)+theta_cmd_1_1SEDA(:,2) - 1.26 ,'b', 'LineWidth', 1.5, 'DisplayName', 'SEDA');
legend show; 
xlabel('Time [s]', 'FontName', 'Times New Roman');
ylabel('error[rad]', 'FontName', 'Times New Roman');
xlim([0 10]);
legend show; 
legend('Orientation','horizontal')

subplot(2,2,2)
title('Drone Roll', 'FontName', 'Times New Roman')%,'FontSize',30)
hold on
grid on
plot(data_log_time_1_1SEA.Time-time_origin_1_1SEA - 56,inchBase_attitude_1_1SEA(:,1) - 0.03,  'r-', 'LineWidth', 1.5, 'DisplayName', 'SEA');
plot(data_log_time_1_1SEDA.Time-time_origin_1_1SEDA - 47 - 0.141,inchBase_attitude_1_1SEDA(:,1) - 0.03,  'b', 'LineWidth', 1.5, 'DisplayName', 'SEDA');
legend show; 
xlabel('Time [s]', 'FontName', 'Times New Roman');
ylabel('error[rad]', 'FontName', 'Times New Roman');
xlim([0 10]);
ylim([-0.1 0.1]);
legend show; 
legend('Orientation','horizontal')

subplot(2,2,4)
title('Drone Pitch', 'FontName', 'Times New Roman')%,'FontSize',30)
hold on
grid on
plot(data_log_time_1_1SEA.Time-time_origin_1_1SEA - 56,inchBase_attitude_1_1SEA(:,2),  'r-', 'LineWidth', 1.5, 'DisplayName', 'SEA');
plot(data_log_time_1_1SEDA.Time-time_origin_1_1SEDA - 47 - 0.141,inchBase_attitude_1_1SEDA(:,2),  'b', 'LineWidth', 1.5, 'DisplayName', 'SEDA');
legend show; 
xlabel('Time [s]', 'FontName', 'Times New Roman');
ylabel('error[rad]', 'FontName', 'Times New Roman');
xlim([0 10]);
ylim([-0.1 0.1]);
legend show; 
legend('Orientation','horizontal')

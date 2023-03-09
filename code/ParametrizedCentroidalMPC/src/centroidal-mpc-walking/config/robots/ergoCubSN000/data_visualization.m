clc;
clearvars;
close all;


load("robot_logger_device_2023_02_14_12_27_28.mat");
jointPositions = robot_logger_device.joints_state.positions.data;
desiredPositions = robot_logger_device.walking.joints_state.positions.desired.data;
measuredPositionWalking = robot_logger_device.walking.joints_state.positions.measured.data;


time = 0:length(desiredPositions(1,1,:))-1;

figure("Name","Torso position tracking")
subplot(2,1,1)
plot(time, desiredPositions(1, :), 'k', 'LineWidth', 2);
hold on; grid on;
% plot(time, jointPositions(1, :), 'r-', 'LineWidth', 2);
plot(time, measuredPositionWalking(1, :), 'r--', 'LineWidth', 2);

l = legend('Desired ',  'Measured Walking');
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('BZ_joint [rad]'); 
set(l,'Interpreter','Latex');
l.FontSize = 30;

subplot(2,1,2)
plot(time, desiredPositions(2, :), 'k', 'LineWidth', 2);
hold on; grid on;
% plot(time, jointPositions(2, :), 'r-', 'LineWidth', 2);
plot(time, measuredPositionWalking(2, :), 'r--', 'LineWidth', 2);

l = legend('Desired ', 'Measured Walking');
set(l,'Interpreter','Latex ');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('BY_joint [rad]'); 
set(l,'Interpreter','Latex');
l.FontSize = 30;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


figure("Name","Left leg position tracking")
subplot(3,2,1)
plot(time, desiredPositions(13, :),  'LineWidth', 2);
hold on; grid on;
% plot(time, jointPositions(1, :), 'r-', 'LineWidth', 2);
plot(time, measuredPositionWalking(13, :),  'LineWidth', 2);

l = legend('Desired ',  'Measured Walking');
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('LLeg_1Z_joint [rad]'); 
set(l,'Interpreter','Latex');
l.FontSize = 15;

subplot(3,2,2)
plot(time, desiredPositions(14, :),  'LineWidth', 2);
hold on; grid on;
% plot(time, jointPositions(2, :), 'r-', 'LineWidth', 2);
plot(time, measuredPositionWalking(14, :),  'LineWidth', 2);

l = legend('Desired ', 'Measured Walking');
set(l,'Interpreter','Latex ');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('LLeg_1X_joint [rad]'); 
set(l,'Interpreter','Latex');
l.FontSize = 15;

subplot(3,2,3)
plot(time, desiredPositions(15, :),  'LineWidth', 2);
hold on; grid on;
% plot(time, jointPositions(1, :), 'r-', 'LineWidth', 2);
plot(time, measuredPositionWalking(15, :), 'LineWidth', 2);

l = legend('Desired ',  'Measured Walking');
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('LLeg_1Y_joint [rad]'); 
set(l,'Interpreter','Latex');
l.FontSize = 15;

subplot(3,2,4)
plot(time, desiredPositions(16, :),  'LineWidth', 2);
hold on; grid on;
% plot(time, jointPositions(2, :), 'r-', 'LineWidth', 2);
plot(time, measuredPositionWalking(16, :),  'LineWidth', 2);

l = legend('Desired ', 'Measured Walking');
set(l,'Interpreter','Latex ');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('LLeg_2Y_joint [rad]'); 
set(l,'Interpreter','Latex');
l.FontSize = 15;


subplot(3,2,5)
plot(time, desiredPositions(17, :),  'LineWidth', 2);
hold on; grid on;
% plot(time, jointPositions(1, :), 'r-', 'LineWidth', 2);
plot(time, measuredPositionWalking(17, :), 'LineWidth', 2);

l = legend('Desired ',  'Measured Walking');
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('LLeg_3Y_joint [rad]'); 
set(l,'Interpreter','Latex');
l.FontSize = 15;

subplot(3,2,6)
plot(time, desiredPositions(18, :),  'LineWidth', 2);
hold on; grid on;
% plot(time, jointPositions(2, :), 'r-', 'LineWidth', 2);
plot(time, measuredPositionWalking(18, :),  'LineWidth', 2);

l = legend('Desired ', 'Measured Walking');
set(l,'Interpreter','Latex ');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('LLeg_3X_joint [rad]'); 
set(l,'Interpreter','Latex');
l.FontSize = 15;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure("Name","Right leg position tracking")
subplot(3,2,1)
plot(time, desiredPositions(19, :),  'LineWidth', 2);
hold on; grid on;
% plot(time, jointPositions(1, :), 'r-', 'LineWidth', 2);
plot(time, measuredPositionWalking(19, :),  'LineWidth', 2);

l = legend('Desired ',  'Measured Walking');
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('LLeg_1Z_joint [rad]'); 
set(l,'Interpreter','Latex');
l.FontSize = 15;

subplot(3,2,2)
plot(time, desiredPositions(20, :),  'LineWidth', 2);
hold on; grid on;
% plot(time, jointPositions(2, :), 'r-', 'LineWidth', 2);
plot(time, measuredPositionWalking(20, :),  'LineWidth', 2);

l = legend('Desired ', 'Measured Walking');
set(l,'Interpreter','Latex ');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('LLeg_1X_joint [rad]'); 
set(l,'Interpreter','Latex');
l.FontSize = 15;

subplot(3,2,3)
plot(time, desiredPositions(21, :),  'LineWidth', 2);
hold on; grid on;
% plot(time, jointPositions(1, :), 'r-', 'LineWidth', 2);
plot(time, measuredPositionWalking(21, :), 'LineWidth', 2);

l = legend('Desired ',  'Measured Walking');
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('LLeg_1Y_joint [rad]'); 
set(l,'Interpreter','Latex');
l.FontSize = 15;

subplot(3,2,4)
plot(time, desiredPositions(22, :),  'LineWidth', 2);
hold on; grid on;
% plot(time, jointPositions(2, :), 'r-', 'LineWidth', 2);
plot(time, measuredPositionWalking(22, :),  'LineWidth', 2);

l = legend('Desired ', 'Measured Walking');
set(l,'Interpreter','Latex ');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('LLeg_2Y_joint [rad]'); 
set(l,'Interpreter','Latex');
l.FontSize = 15;


subplot(3,2,5)
plot(time, desiredPositions(23, :),  'LineWidth', 2);
hold on; grid on;
% plot(time, jointPositions(1, :), 'r-', 'LineWidth', 2);
plot(time, measuredPositionWalking(23, :), 'LineWidth', 2);

l = legend('Desired ',  'Measured Walking');
set(l,'Interpreter','Latex');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('LLeg_3Y_joint [rad]'); 
set(l,'Interpreter','Latex');
l.FontSize = 15;

subplot(3,2,6)
plot(time, desiredPositions(24, :),  'LineWidth', 2);
hold on; grid on;
% plot(time, jointPositions(2, :), 'r-', 'LineWidth', 2);
plot(time, measuredPositionWalking(24, :),  'LineWidth', 2);

l = legend('Desired ', 'Measured Walking');
set(l,'Interpreter','Latex ');
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex');
l = ylabel('LLeg_3X_joint [rad]'); 
set(l,'Interpreter','Latex');
l.FontSize = 15;
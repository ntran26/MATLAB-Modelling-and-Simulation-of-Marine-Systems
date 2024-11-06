clear, clc, close all;

% Load autopilot data
load SpeedPIDdata.mat

% Assign data
time = data(:,11);
surge = data(:,1);
sway = data(:,2);
yaw_rate = data(:,3);
x_pos = data(:,4);
y_pos = data(:,5);
yaw = data(:,6);
roll_rate = data(:,7);
roll = data(:,8);
rudder = data(:,9);
shaft = data(:,10);

desired_speed = 10;

% Display data
figure;
subplot(2,1,1);
plot(time, surge, 'LineWidth', 1.5);
yline(desired_speed, 'r--', 'Desired Speed');
grid on;
ylabel("Speed [m/s]");
title("Speed Control with PID");

subplot(2,1,2);
plot(time, shaft, 'LineWidth', 1.5);
grid on;
ylabel("Shaft [m/s]");
xlabel("Time [s]");

figure
subplot(3,1,1)
plot(time, yaw*180/pi, "LineWidth", 1.5);
yline(20, 'r--', 'Desired Course');
grid on;
ylabel("Heading [deg]");

subplot(3,1,2)
plot(time, rudder*180/pi, "LineWidth", 1.5);
grid on;
ylabel("Rudder [deg]");

subplot(3,1,3)
plot(time,yaw_rate*180/pi, "LineWidth", 1.5);
grid on;
ylabel("Yaw rate [deg/s]");

figure
plot(y_pos, x_pos);
title("Trajectory")
axis("equal");
grid on;
ylabel("X-position [m]");
xlabel("Y-position [m]");
% Module 6 Activity 1
% Ship Hull Dynamics and Position

clear, clc, close all;

% Initial condition
x = [0; 0];

% Input vector
u = [30*pi/180; 0];

% Simulation parameters
step = 0.1;
ts = 0;
tf = 100;
index = 0;

for t = ts:step:tf
    index = index + 1;
    k1 = step * ship_model(x,u);
    k2 = step * ship_model(x+k1, u);
    x = x + (k1 + k2)/2;

    % Set boundary for yaw
    if x(1) > 2*pi
        x(1) = x(1) - 2*pi;
    elseif x(1) <= 0
        x(1) = x(1) + 2*pi;
    else
        x(1) = x(1);
    end

    % Store data
    data(index, 1) = t;         % time (s)
    data(index, 2) = x(1);      % yaw angle (rad)
    data(index, 3) = x(2);      % yaw rate (rad/s)
    data(index, 4) = u(1);      % rudder angle (rad)
end

% Plotting
subplot(3,1,1);
plot(data(:,1), data(:,2)*180/pi, 'LineWidth', 2);
grid on;
title('Ploting yaw, yaw rate, rudder');
ylabel('Yaw [deg]');

subplot(3,1,2);
plot(data(:,1), data(:,3)*180/pi, 'LineWidth', 2);
grid on;
ylabel('Yaw rate [deg/s]');

subplot(313);
plot(data(:,1), data(:,4)*180/pi, 'LineWidth', 2);
grid on;
ylabel('Rudder [deg]');
xlabel('Time [sec]');

function x_dot = ship_model(x, u)
    Iz = 15e9;      % kgm^2
    N_del = 22e7;   % Nm/rad
    Nr = 2e9;       % Nms/rad

    T = Iz/Nr;
    K = N_del/Nr;

    psi = x(1);
    r = x(2);
    del = u(1);

    x_dot = [r
             (K/T)*del - (r/T)];
end
% 
% function [xdot, Y] = activity1(x,u,t)
%     T = 7.5;
%     K = 0.11;
% 
%     xdot = [x(2)
%             (-1/T)*x(2) + (K/T)*u(1)];
% 
%     Y = [1 0;0 1][x(1);x(2)] + [0;0]*[u(1)];
% end
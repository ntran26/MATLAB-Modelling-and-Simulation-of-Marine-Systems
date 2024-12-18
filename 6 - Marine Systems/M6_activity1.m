% Open-loop system without trajectory

clear, clc, close all;

% Initial condition and inputs
x = [0;0];
del = 30*pi/180;    % rudder angle
u = [del];

% Simulation parameter
dt = 0.05;
ft = 400;

% Solver using Euler
index = 0;
for ii = 0:dt:ft
    index = index + 1;
    xdot = M6_A1(x,u,ii);
    x = x + dt*xdot;

    % set 0 <= x(1) <= 360
    if x(1) > 2*pi
        x(1) = x(1) - 2*pi;
    elseif x(1) < 0;
        x(1) = x(1) + 2*pi;
    else
        x(1) = x(1);
    end

    % Store simulated data
    data(index,1) = ii;             % time [s]
    data(index,2) = u(1)*180/pi;    % rudder [deg]
    data(index,3) = x(1)*180/pi;    % yaw [deg]
    data(index,4) = x(2)*180/pi;    % yaw rate [deg/s]

end

% Plotting
t = data(:,1);
rudder = data(:,2);
yaw = data(:,3);
yaw_rate = data(:,4);

figure
subplot(3,1,1);
plot(t,rudder);
title("Rudder angle over time");
ylabel("r [deg]")
grid on;

subplot(3,1,2);
plot(t,yaw);
title("Yaw angle over time");
ylabel("\psi [deg]")
grid on;

subplot(3,1,3);
plot(t,yaw_rate);
title("Yaw rate over time");
ylabel("$\dot{\psi}$ [deg]", 'Interpreter','latex')
grid on;
xlabel("Time [s]")

function [xdot, Y] = M6_A1(x, u, t)
    T = 7.5;
    K = 0.11;
    
    % State equation
    xdot = [x(2)
            -1/T*x(2) + K/T*u(1)];

    % Output equation
    Y = [1 0;0 1]*[x(1);x(2)] + [0;0]*[u(1)];

end
% Simulation of open-loop systems with steering machine

clear, clc, close all;

% Initial condition and inputs
x = [0;0;0;0;0];
delc = 30*pi/180;    % commanded rudder angle
u = [delc];

% Simulation parameter
dt = 0.05;
ft = 400;

% Solver using Euler
index = 0;
for ii = 0:dt:ft
    index = index + 1;
    xdot = M6_A4(x,u,ii);
    x = x + dt*xdot;

    % set 0 <= x(1) <= 360
    if x(2) > 2*pi
        x(2) = x(2) - 2*pi;
    elseif x(2) < 0
        x(2) = x(2) + 2*pi;
    else
        x(2) = x(2);
    end

    % Store simulated data
    data(index,1) = ii;             % time [s]
    data(index,2) = u(1)*180/pi;    % commanded rudder [deg]
    data(index,3) = x(1)*180/pi;    % actual rudder [deg]
    data(index,4) = x(2)*180/pi;    % yaw angle [deg]
    data(index,5) = x(3)*180/pi;    % yaw rate [deg/s]
    data(index,6) = x(4);           % x position [m]
    data(index,7) = x(5);           % y position [m]

end

% Extract data
t = data(:,1);
command_rudder = data(:,2);
actual_rudder = data(:,3);
yaw = data(:,4);
yaw_rate = data(:,5);
xpos = data(:,6);
ypos = data(:,7);

% Plotting
figure
subplot(3,1,1);
plot(t,command_rudder, "LineWidth", 2);
hold on
plot(t,actual_rudder, "LineWidth", 1);
title("Rudder angle over time");
ylim([0 40]);
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

figure
plot(ypos,xpos);
grid on;
axis("equal")
title("Trajectory");

% State space model to State vector:
% x(1) = rudder (u)
% x(2) = yaw
% x(3) = yaw rate
% x(4) = x-position
% x(5) = y-position

function [xdot, Y] = M6_A4(x, u, t)
    T = 7.5;
    K = 0.11;
    ui = 12*1850/3600;   % surge velocity [m/s]
    vi = 0*1850/3600;    % sway velocity [m/s]
    Trud = 11.9;         % rudder time constant
    a = 1;               % constant

    % Commanded rudder angle
    delc = u(1);
    
    % State equation
    xdot = [(delc - x(1)) / (abs(delc - x(1))*Trud + a) 
            x(3)
            -1/T*x(3) + K/T*x(1)
            ui*cos(x(2)) + vi*sin(x(2))
            ui*sin(x(2)) - vi*cos(x(2))];

    % Output equation
    Y = [1 0;0 1]*[x(1);x(2)] + [0;0]*[u(1)];
end
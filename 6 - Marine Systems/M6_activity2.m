% Simulation of the open-loop with trajectory (turning circle)

clear, clc, close all;

% Initial condition and inputs
x = [0;0;0;0];
del = 30*pi/180;    % rudder angle
u = [del];

% Simulation parameter
dt = 0.05;
ft = 400;

% Solver using Euler
index = 0;
for ii = 0:dt:ft
    index = index + 1;
    xdot = M6_A2(x,u,ii);
    x = x + dt*xdot;

    % set 0 <= x(1) <= 360
    if x(1) > 2*pi
        x(1) = x(1) - 2*pi;
    elseif x(1) < 0
        x(1) = x(1) + 2*pi;
    else
        x(1) = x(1);
    end

    % Store simulated data
    data(index,1) = ii;             % time [s]
    data(index,2) = u(1)*180/pi;    % rudder [deg]
    data(index,3) = x(1)*180/pi;    % yaw [deg]
    data(index,4) = x(2)*180/pi;    % yaw rate [deg/s]
    data(index,5) = x(3);           % x position [m]
    data(index,6) = x(4);           % y position [m]

end

% Extract data
t = data(:,1);
rudder = data(:,2);
yaw = data(:,3);
yaw_rate = data(:,4);
xpos = data(:,5);
ypos = data(:,6);

% Plotting
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

figure
plot(ypos,xpos);
grid on;
axis("equal")
title("Trajectory");

function [xdot, Y] = M6_A2(x, u, t)
    T = 7.5;
    K = 0.11;
    ui = 12*1850/3600;   % surge velocity [m/s]
    vi = 0*1850/3600;    % sway velocity [m/s]
    
    % State equation
    xdot = [x(2)
            -1/T*x(2) + K/T*u(1)
            ui*cos(x(1)) + vi*sin(x(1))
            ui*sin(x(1)) - vi*cos(x(1))];

    % Output equation
    Y = [1 0;0 1]*[x(1);x(2)] + [0;0]*[u(1)];

end

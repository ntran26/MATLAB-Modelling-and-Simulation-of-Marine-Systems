% Simulation of closed-loop system with PID autopilot

clear, clc, close all;

% Initial condition
x = [0;0;0;0;0];

% Desired yaw angle
psid = 5*pi/180;

% PID control gains and initial settings
K = [0.75 0.025 5];
ei = 0;
ed0 = 0;
e = [0; ei; ed0];

% Gyrocompass sensitivity
Hgc = 1;    % V/rad

% Simulation parameter
dt = 0.1;
ft = 400;

% Solver using RK2
index = 0;
for ii = 0:dt:ft
    index = index + 1;
    
    % Recall PIDAutopilot function
    u = PIDAutopilot(K,e);

    k1 = dt*M6_A5(x, u, ii);
    k2 = dt*M6_A5(x + k1, u, ii);
    x = x + 0.5*(k1+k2);

    % Boundary of yaw angle (0 to 360)
    if x(1) > 2*pi
        x(1) = x(1) - 2*pi;
    elseif x(1) < 0
        x(1) = x(1) + 2*pi;
    else
        x(1) = x(1);
    end

    % Gyrocompass
    psim = x(1)*Hgc;    % measured course in volts
    
    % Error between psid and psim
    error = psid - psim;

    % Limit for error
    if error >= pi
        error = error - 2*pi;
    elseif error <= -pi
        error = error + 2*pi;
    else
        error = error;
    end

    % Compute P,I,D terms
    e(1) = error;
    k11 = dt*error;
    k12 = dt*(error + k11);
    ei = ei + 0.5*(k11 + k12);
    e(2) = ei;
    ed = (error - ed0)/dt;
    e(3) = ed;
    ed0 = error;

    % Store simulated data
    data(index,1) = ii;             % time [s]
    data(index,2) = x(1)*180/pi;    % yaw [deg]
    data(index,3) = x(2)*180/pi;    % yaw rate [deg]
    data(index,4) = x(3)*180/pi;    % rudder [deg/s]
    data(index,5) = x(4);           % x position [m]
    data(index,6) = x(5);           % y position [m]

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
plot(t,rudder, "LineWidth", 2);
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

% State space model to State vector:
% x(1) = rudder (u)
% x(2) = yaw
% x(3) = yaw rate
% x(4) = x-position
% x(5) = y-position

function [xdot, Y] = M6_A5(x, u, t)
    T = 7.5;
    K = 0.11;
    ui = 12*1850/3600;   % surge velocity [m/s]
    vi = 0*1850/3600;    % sway velocity [m/s]
    Trud = 11.9;         % rudder time constant
    a = 1;               % constant

    psi = x(1);
    r = x(2);
    del = x(3);
    del_c = u(1);
    
    % Return Derivatives;
    
    xdot = [r
            -r/T + (K/T) * del
            (del_c-del)/(abs(del_c-del)*Trud + a)
            ui*cos(psi) - vi*sin(psi)
            ui*sin(psi) + vi*cos(psi)];

    % Output equation
    Y = [1 0;0 1]*[x(1);x(2)] + [0;0]*[u(1)];
end

function u = PIDAutopilot(K, e)
    % Compute to PID control signal and set the upper/lower limit
    % for the PID controller
    Kp = K(1);
    Ki = K(2);
    Kd = K(3);
    limit = 10*pi/180;  % limit in rad

    upid = e(1)*Kp + e(2)*Ki + e(3)*Kd;

    % limits
    if upid >= limit
        upid = limit;
    elseif upid <= -limit
        upid = -limit;
    else
        upid = upid;
    end

    % Return the PID control panel
    u = [upid
            0];
end
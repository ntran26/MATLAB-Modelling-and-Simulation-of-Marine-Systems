% Simulation of open-loop systems with steering machine


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
    
    
    % State equation
    xdot = [x(2)
            -1/T*x(2) + K/T*u(1)
            ui*cos(x(1)) + vi*sin(x(1))
            ui*sin(x(1)) - vi*cos(x(1))];

    % Output equation
    Y = [1 0;0 1]*[x(1);x(2)] + [0;0]*[u(1)];
end
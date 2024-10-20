% Open-loop system without trajectory

clear, clc, close all;

% Initial condition and input
x = [0;0];
del = 30*pi/180;    % rudder angle
u = [del];

% Simulation parameters
dt = 0.05;
ft = 400;

% Solver using Euler method
index = 0;
for ii = 0:dt:ft
    index = index + 1;
    xdot = M6_A1(x,u,ii);
    x = x + dt*xdot;
end

% Plotting
figure


% Function
function [xdot,Y] = M6_A1(x,u,t)
    T = 7.5;
    K = 0.11;
        
    xdot = [x(2)
            -1/T*x(2) + K/T*u(1)];

    Y = [1 0;0 1][x(1);x(2)] + [0;0]*[u(1)];
    

end
clear, clc, close all;

% Time span
ts = 0;
tf = 50;

% Numerical solution
h = 0.1;
x = [5; -2]*pi/180;

i = 0;

for k = 0:h:tf
    i = i+1;
    x_dot = pendulum(k,x);
    data(i,1) = k;
    data(i,2) = x(1);
    data(i,3) = x(2);
end

plot(data(:,1), data(:,2), data(:,1), data(:,3));

% Function for pendulum system without exiting force
function x_dot = pendulum(t,x)
    
    g = 9.81;   % gravity
    L = 10;     % length of pendulum

    x_dot = [x(2)
             -g/L*sin(x(1))];

end

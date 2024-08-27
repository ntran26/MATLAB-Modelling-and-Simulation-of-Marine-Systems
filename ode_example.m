clc, clear, close all;

% Time span
tspan = [0 20];         % 0 to 20 seconds

% Initial condition
x10 = [-1; 2];
x10 = [0; 0];

% Solving
[t1, y1] = ode45(@activity1_eq2, tspan, x10);

% Calculate the exact solution
t = 0:0.01:20;
y_ex1 = 5/2 - 5*exp(-t) + 3/2*exp(-2*t);
y_ex2 = 1 - (1/4.*exp(-t) .* (4.*cos(2.*t) + 2.*sin(2.*t)));

% Plotting
figure
plot(t1,y1(:,1), 'b', t, y_ex2, '--r');

% Second method (using loop)
i = 0;
for k = tspan(1):0.01:tspan(2)
    i = i + 1;
    y_ex3 = 1 - 0.25*exp(-k) * (4*cos(2*k) + 2*sin(2*k));
    data(i,1) = k;
    data(i,2) = y_ex3;
end

figure
plot(t1,y1(:,1), 'b', data(:,1), data(:,2), '--r');
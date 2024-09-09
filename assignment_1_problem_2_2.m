% Author: Nam Tran
% This program simulates the fluid storage level control system
% It uses built-in solver and Euler's method to compare
% the effects of differnt Kv constant to the water level
% It also shows the behaviour of inlet flowrate and fluid level with a
% fixed input voltage value

clear, clc, close all;

% Parameters
A = 0.0254;         % Cross-sectional area
Ku = 0.327;         % Constant between input voltage and inlet flowrate
Kv1 = 0.0015/A;     % Original Kv value
Kv2 = 0.001/A;      % Alternate Kv value 1
Kv3 = 0.002/A;      % Alternate Kv value 2
tspan = [0 100];    % Timespan 100 seconds
h0 = 0;             % Initial fluid level
u1 = 5;             % Input voltage

% Solve for Kv1 using ode45 (built-in solver)
[t, h1] = ode45(@(t, h) problem2(t, h, u1, Ku, Kv1), tspan, h0);

% Solve for Kv2 and Kv3 using ode45
[t, h2] = ode45(@(t, h) problem2(t, h, u1, Ku, Kv2), tspan, h0);
[t, h3] = ode45(@(t, h) problem2(t, h, u1, Ku, Kv3), tspan, h0);

% Plot Inlet Flow Rate and Control Signal
% Ku = 30 (L/h/V) => qin (L/h)
qin = 30 * u1;

figure
subplot(3,1,1);
plot(t, qin * ones(size(t)), 'LineWidth', 2);
% xlabel('Time (s)');
ylabel('Inlet Flowrate (L/h)');
title('Inlet Flow Rate');
grid on;

subplot(3,1,2);
plot(t, u1 * ones(size(t)), 'LineWidth', 2);
% xlabel('Time (s)');
ylabel('Control Signal (V)');
title('Control Signal');
grid on;

% Plot Fluid Level and Feedback Signal for Kv1
subplot(3,1,3);
plot(t, h1, 'LineWidth', 2); hold on;
plot(t, h1, '--', 'LineWidth', 2); % Feedback is same as h1
xlabel('Time (s)');
ylabel('Level (mm)');
title('Fluid Level and Feedback Signal');
legend('Level', 'Feedback Signal', 'Location', 'northwest');
grid on;
hold off;

% Compare Fluid Levels for Different Kv
figure
subplot(2,1,1);
plot(t, h1, 'LineWidth', 2); hold on;
plot(t, h2, 'LineWidth', 2);
plot(t, h3, 'LineWidth', 2);
% xlabel('Time (s)');
ylabel('Fluid Level (mm)');
title('Fluid Level Comparison for Different Kv - Built-in Solver');
legend('Kv = 0.0015', 'Kv = 0.001', 'Kv = 0.002', 'Location', 'northwest');
grid on;
hold off;

% % Euler's Method for Kv1
% h_euler = 0;                % initial fluid level
% tf = 100;                   % final time
% h = 0.01;                   % step size
% n = tf/h;                   % number of steps 
% h_vals = zeros(1, n);       % Fluid level
% P_vals = zeros(1, n);       % Pressure
% t_euler = linspace(0, tf, n);
% 
% % Euler's method loop
% for k = 1:n-1
%     h_dot = Ku * u1 - Kv1 * h_euler;
%     h_euler = h_euler + h * h_dot;
%     h_vals(k+1) = h_euler;
%     P_vals(k+1) = rho_g * h_euler;
% end
% 
% % Plot Euler Method Results for Kv1
% figure;
% subplot(2,1,1);
% plot(t_euler, h_vals, 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Fluid Level (m)');
% title('Fluid Level - Euler Method for Kv = 0.0591');
% grid on;
% 
% subplot(2,1,2);
% plot(t_euler, P_vals, 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Pressure P(t) (Pa)');
% title('Pressure - Euler Method for Kv = 0.0591');
% grid on;

% Euler's Method for Kv1, Kv2, Kv3
tf = 100;
h = 0.01;
n = tf/h;    
t_euler = linspace(0, tf, n);   % time vector 

% Preallocate for each Kv
h_vals1 = zeros(1, n); 
h_vals2 = zeros(1, n); 
h_vals3 = zeros(1, n);

% Initial conditions
h_euler1 = 0; 
h_euler2 = 0;
h_euler3 = 0; 

% Euler method loop for Kv1, Kv2, Kv3
for k = 1:n-1
    % Kv1
    h_dot1 = Ku * u1 - Kv1 * h_euler1;
    h_euler1 = h_euler1 + h * h_dot1;
    h_vals1(k+1) = h_euler1;
    
    % Kv2
    h_dot2 = Ku * u1 - Kv2 * h_euler2;
    h_euler2 = h_euler2 + h * h_dot2;
    h_vals2(k+1) = h_euler2;
    
    % Kv3
    h_dot3 = Ku * u1 - Kv3 * h_euler3;
    h_euler3 = h_euler3 + h * h_dot3;
    h_vals3(k+1) = h_euler3;
end

% Plot Comparison Using Euler's Method
subplot(2,1,2);
plot(t_euler, h_vals1, 'LineWidth', 2); hold on;
plot(t_euler, h_vals2, 'LineWidth', 2); hold on;
plot(t_euler, h_vals3, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Fluid Level (mm)');
title('Fluid Level Comparison for Different Kv - Euler Method');
legend('Kv = 0.0015', 'Kv = 0.001', 'Kv = 0.002', 'Location', 'northwest');
grid on;
hold off;

% Function for level dynamics
function h_dot = problem2(t, h, u1, Ku, Kv)
    % Inlet flow rate due to pump voltage
    qin = Ku * u1;
    % Outlet flow rate, proportional to the level h
    qout = Kv * h;
    % Differential equation: rate of change of fluid level
    h_dot = qin - qout;
end

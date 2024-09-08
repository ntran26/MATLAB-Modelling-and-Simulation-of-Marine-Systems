clear, clc, close all;

% Parameters
Ku = 0.328;         % Constant between input voltage and inlet flowrate
Kv = 0.0591;        % Outlet flowrate constant
rho_g = 9810;       % Pressure factor (1000*9.81)

tspan = [0 100];    % timespan 100 seconds
h0 = 0;             % initial fluid level
u1 = 5;             % input voltage

% Solve the differential equation using ode45
[t, h] = ode45(@(t, h) level_dynamics(t, h, u1, Ku, Kv), tspan, h0);

% Calculate the pressure at each time step
P = rho_g * h;

% Plot the fluid level over time
figure;
subplot(2, 1, 1);
plot(t, h, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Fluid Level (m)');
title('Fluid Level - Built-in Solver');
grid on;

% Plot the pressure over time
subplot(2, 1, 2);
plot(t, P, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Pressure (Pa)');
title('Pressure - Built-in Solver');
grid on;

% Euler's Method
h_euler = 0;                % initial fluid level
tf = 100;                   % final time
h_step = 0.01;              % step size
n = round(tf / h_step);     % number of steps 
t_euler = linspace(0, tf, n);

h_vals = zeros(1, n);       % fluid level
P_vals = zeros(1, n);       % pressure

% Euler method loop
for k = 1:n-1
    h_dot = Ku * u1 - Kv * h_euler;
    h_euler = h_euler + h_step * h_dot;
    h_vals(k+1) = h_euler;  
    P_vals(k+1) = rho_g * h_euler;  
end

% Plot Euler method results
figure;
subplot(2,1,1);
plot(t_euler, h_vals, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Fluid Level h(t) (m)');
title('Fluid Level - Euler Method');
grid on;

subplot(2,1,2);
plot(t_euler, P_vals, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Pressure P(t) (Pa)');
title('Pressure - Euler Method');
grid on;



% Function representing the level dynamics (differential equation)
function solver = level_dynamics(t, h, u1, Ku, Kv)
    % Inlet flow rate due to pump voltage
    qin = Ku * u1;
    
    % Outlet flow rate, proportional to the level h
    qout = Kv * h;
    
    % Differential equation: rate of change of fluid level
    solver = qin - qout;
end

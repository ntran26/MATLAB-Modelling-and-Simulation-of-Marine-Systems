clear, clc, close all;

% Parameters for the system
Ku = 0.328; % Proportional constant for input voltage to level dynamics
Kv1 = 0.0591; % Original value of Kv (m^2/s)
Kv2 = 0.05; % Alternative value of Kv (m^2/s)
Kv3 = 0.07; % Alternative value of Kv (m^2/s)
rho_g = 9810; % Factor for converting level to pressure (1000 * 9.81)
max_flow_rate = 300; % Maximum flow rate in liters per hour

% Time span for simulation
tspan = [0 100];  % Simulate for 100 seconds

% Initial fluid level
h0 = 0;  % Initial level (0 meters)

% Input voltage u1 (control signal)
u1 = 5; % Constant input voltage in Volts

% Control signal (same as u1)
uc = u1;

% Calculate inlet flow rate qin(t) in liters per hour
qin = @(t) Ku * u1 * 3600; % Convert m^3/s to liters/hour

% Simulations with different Kv values
[t1, h1] = ode45(@(t, h) level_dynamics(t, h, u1, Ku, Kv1), tspan, h0);
[t2, h2] = ode45(@(t, h) level_dynamics(t, h, u1, Ku, Kv2), tspan, h0);
[t3, h3] = ode45(@(t, h) level_dynamics(t, h, u1, Ku, Kv3), tspan, h0);

% Compute feedback signal for each Kv (h_m = h in this case)
hm1 = h1; hm2 = h2; hm3 = h3;

% Plot inlet flow rate
figure;
subplot(3,1,1);
plot(t1, qin(t1), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Inlet Flow Rate q_{in} (liters/hour)');
title('Inlet Flow Rate vs Time');

% Plot control signal
subplot(3,1,2);
plot(t1, u1 * ones(size(t1)), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Control Signal u_c (V)');
title('Control Signal vs Time');

% Plot level h(t) and feedback signal hm(t) for Kv1
subplot(3,1,3);
plot(t1, h1, 'LineWidth', 2); hold on;
plot(t1, hm1, '--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Level h(t) (m)');
title('Fluid Level and Feedback Signal for K_v = 0.0591');
legend('Level h(t)', 'Feedback Signal h_m(t)');
hold off;

% Compare fluid levels for different Kv values
figure;
plot(t1, h1, 'LineWidth', 2); hold on;
plot(t2, h2, 'LineWidth', 2);
plot(t3, h3, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Fluid Level h(t) (m)');
title('Fluid Level Comparison for Different K_v');
legend('K_v = 0.0591', 'K_v = 0.05', 'K_v = 0.07');
hold off;

% Plot level h(t) and feedback signal hm(t) for Kv2 and Kv3
figure;
subplot(2,1,1);
plot(t2, h2, 'LineWidth', 2); hold on;
plot(t2, hm2, '--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Level h(t) (m)');
title('Fluid Level and Feedback Signal for K_v = 0.05');
legend('Level h(t)', 'Feedback Signal h_m(t)');
hold off;

subplot(2,1,2);
plot(t3, h3, 'LineWidth', 2); hold on;
plot(t3, hm3, '--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Level h(t) (m)');
title('Fluid Level and Feedback Signal for K_v = 0.07');
legend('Level h(t)', 'Feedback Signal h_m(t)');
hold off;

% Function for fluid level dynamics
function h_dot = level_dynamics(t, h, u1, Ku, Kv)
    h_dot = Ku * u1 - Kv * h;  % Differential equation
end

clear, clc, close all

% Parameters
R = 0.25;       % Resistance (Ohms)
L = 0.005;      % Inductance 
Kt = 36.3e-3;   % Torque constant (Nm/A)
Ke = 1/Kt;      % Back EMF constant (V/(rad/s))
J = 1.68e-6;    % Rotor Inertia (kg.m^2)
B = 1e-6;       % Damping coefficient (N.m.s/rad, estimated)
Va = 48;        % Applied voltage (V)

% Transfer function between shaft speed and input voltage
num = [Kt];
den = [J*L, (B*L + R*J), (B*R + Kt*Ke)];
sys_speed = tf(num, den);

% Transfer function between shaft angle and input voltage
sys_angle = tf(num, [den 0]);

% Simulation time
t = 0:0.001:1;

% Step response
figure;
subplot(2,1,1);
step(sys_speed, t);
title('Step Response: Shaft Speed vs Input Voltage');
ylabel('Speed (rad/s)');
xlabel('Time (s)');

subplot(2,1,2);
step(sys_angle, t);
title('Step Response: Shaft Angle vs Input Voltage');
ylabel('Angle (rad)');
xlabel('Time (s)');

% State-space representation
A = [-B/J, Kt/J; -Ke/L, -R/L];
B = [0; 1/L];
C = [1, 0];
D = [0];
sys_ss = ss(A, B, C, D);

% Time response for a step input
[y, t, x] = step(sys_ss, t);

figure;
plot(t, y);
title('State-Space Response: Shaft Speed');
xlabel('Time (s)');
ylabel('Speed (rad/s)');
grid on;

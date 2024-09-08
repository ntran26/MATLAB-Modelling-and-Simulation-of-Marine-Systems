% Author: Nam Tran
% This program plots the armature current, speed, shaft angle over time.
% Since the input voltage is constant, we don't plot it.
% This program uses 2 methods: Built-in ODE solver and Euler's method.

clear, clc, close all

% Built-in solver
tspan = [0 2];     % time span

u = [48; 0];       % input voltage step

% Initial conditions [angle, speed, armature current]
x0 = [0; 0; 0];

% Solve using ode45 (built-in solver)
[t, x] = ode45(@problem1, tspan, x0, [], u);

% Plot results
figure;
subplot(3,1,1);
plot(t, x(:,3), "LineWidth", 2); 
% xlabel('Time (s)');
ylabel('Current (A)'); 
title('Armature Current');

subplot(3,1,2);
plot(t, x(:,2), "LineWidth", 2);
% xlabel('Time (s)');
ylabel('Speed (rad/s)');
title('Motor Speed');

subplot(3,1,3);
plot(t, x(:,1), "LineWidth", 2);
xlabel('Time (s)');
ylabel('Angle (rad)');
title('Shaft Angle');





% Euler Method
R = 3.72;       % Resistance (Ohms)
L = 1.04e-3;    % Inductance (H)
K = 0.068;      % Torque constant (Nm/A)
J = 64.5e-7;    % Rotor Inertia (kg.m^2)
b = 1.29e-3;    % Damping coefficient (kgm^2/s, estimated)
Va = 48;        % Input voltage (V)

% Start and final time and step
ts = 0;
tf = 2;
h = 0.0001;
n = tf/h;
tspan = ts:h:tf;

% Initial conditions [angle; speed; current]
x = zeros(3, length(tspan));  % [x1 = angle, x2 = speed, x3 = current]
x(:,1) = [0; 0; 0];

% Euler method loop
for k = 1:n
    x(1,k+1) = x(1,k) + h * x(2,k);     % Update angle
    x(2,k+1) = x(2,k) + h * (-b/J * x(2,k) + K/J * x(3,k));  % Update speed
    x(3,k+1) = x(3,k) + h * (-K/L * x(2,k) - R/L * x(3,k) + Va/L);  % Update current
end

% Plot the results
figure;
subplot(3,1,1);
plot(tspan, x(3,:), "LineWidth", 2);
% xlabel('Time (s)');
ylabel('Current (A)');
title('Armature Current');

subplot(3,1,2);
plot(tspan, x(2,:), "LineWidth", 2);
% xlabel('Time (s)');
ylabel('Speed (rad/s)');
title('Motor Speed');

subplot(3,1,3);
plot(tspan, x(1,:), "LineWidth", 2);
xlabel('Time (s)');
ylabel('Angle (rad)');
title('Shaft Angle');

function xdot = problem1(t,x,u)
    R = 3.72;       % Resistance (Ohms)
    L = 1.04e-3;    % Inductance (H)
    K = 0.068;      % Torque constant (Nm/A)
    J = 64.5e-7;    % Rotor Inertia (kg.m^2)
    b = 1.29e-3;    % Damping coefficient (kgm^2/s, estimated)

    % State equations
    xdot = [x(2);
            -b/J*x(2) + K/J*x(3);
            -K/L*x(2) - R/L*x(3) + 1/L*u(1)];
end


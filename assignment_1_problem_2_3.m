% Author: Nam Tran
% This program is an extension of the fluid storage level control system
% It considers the scenario where there's a leakage slowly drains the water
% It also shows a solution to resolve when there is a leakage to keep the
% desired fluid level, by applying a controlled pumping system

clear, clc, close all;

% Parameters
Kv_leak = 0.01;     % leakage constant
Ku = 0.327;         % pump constant
tspan = [0 500];    % simulate over 500 sec
h0 = 100;           % initial fluid level = 10 cm
h_target = h0;      % target fluid level = 10 cm

% Solve the system for leakage using ode45
[t, h] = ode45(@(t, h) level_dynamics_leak(t, h, Kv_leak), tspan, h0);

% Find the time when the fluid level reaches 1 cm
index_1cm = find(h <= 10, 1);       % find the first index where h <= 10 mm (1 cm)
time_to_1cm = t(index_1cm);        

% Display the result
fprintf('Time taken for the fluid level to drop from 10 cm to 1 cm: %.2f seconds\n', time_to_1cm);

% Plot the fluid level over time
figure;
plot(t, h, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Fluid Level (mm)');
title('Fluid Level Over Time (Leakage Scenario)');
grid on;



% Solve the system with controlled inlet flow to keep level at 10 cm
[t, h] = ode45(@(t, h) level_dynamics_control(t, h, Ku, Kv_leak, h_target), tspan, h0);

% Plot the fluid level over time
figure;
plot(t, h, 'LineWidth', 2);
xlabel('Time (s)');
ylim([0 110]);
ylabel('Fluid Level (mm)');
title('Fluid Level Control with Leakage');
grid on;

% Function for level dynamics with leakage
function solver1 = level_dynamics_leak(t, h, Kv_leak)
    % No inlet flow because the pump is off
    % Leakage flow proportional to fluid level
    qleak = Kv_leak * h;
    
    % Differential equation: rate of change of fluid level
    solver1 = -qleak;
end

% Function for level dynamics with controlled inlet flow
function solver2 = level_dynamics_control(t, h, Ku, Kv_leak, h_target)
    % Control signal
    u1 = 10;

    % Leakage flow proportional to fluid level
    qleak = Kv_leak * h;
    
    % Control strategy: Add just enough pump flow to maintain the level at 10 cm
    if h < h_target
        % If the fluid level is below the target, turn on the pump
        qin = Ku * u1;  
    else
        % If the fluid level is at or above the target, stop the pump
        qin = 0;
    end
    
    % Differential equation: rate of change of fluid level
    solver2 = qin - qleak;
end



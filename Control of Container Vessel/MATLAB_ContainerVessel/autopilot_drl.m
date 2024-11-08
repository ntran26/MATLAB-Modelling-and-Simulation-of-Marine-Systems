clear, clc, close all;

load("AutopilotAgent.mat");
agent = agent1_Trained;

% Initialize parameters
u0 = 8;             
n_ref = 70;
n_c = 70;
setco = 20*pi/180;  % Desired course in radians

% Initial state vector
X = [u0; 0; 0; 0; 0; 0; 0; 0; 0; n_c];
tf = 600;  % Simulation time
dt = 0.1;
index = 0;

% Initialize variables for storing results
time = [];
data = [];

% Simulation loop
for ii = 0:dt:tf
    index = index + 1;

    % Define observation for the RL agent (e.g., current heading error)
    currentYaw = X(6);  % yaw angle in radians
    headingError = wrapToPi(setco - currentYaw);  % [-pi, pi]
    observation = X(9);  

    % Get the rudder action from the RL agent
    % Convert the observation to a format compatible with the agent
    action = getAction(agent, observation);
    action = action{1};
    
    % Extract rudder angle command from action output
    delta_c = action(1);  
    delta_c = max(min(delta_c, 10*pi/180), -10*pi/180);  

    % Define control input vector [delta_c; n_c]
    uu = [delta_c; n_c];

    % Update the state using the dynamics function
    [Xdot, U] = container(X, uu);
    X = X + dt * Xdot;  % Euler integration for state update

    % Boundary of yaw angle
    if X(6) >= 2*pi
        X(6) = X(6) - 2*pi;
    elseif X(6) <= -2*pi
        X(6) = X(6) + 2*pi;
    end

    % Store data
    time(index) = ii;
    data(index,1) = X(1);   % surge velocity [m/s]
    data(index,2) = X(2);   % sway velocity [m/s]
    data(index,3) = X(3);   % yaw rate [rad/s]
    data(index,4) = X(4);   % x-position [m]
    data(index,5) = X(5);   % y-position [m]
    data(index,6) = X(6);   % yaw angle [rad]
    data(index,7) = X(7);   % roll rate [rad/s]
    data(index,8) = X(8);   % roll angle [rad]
    data(index,9) = delta_c; % rudder angle command [rad]
    data(index,10) = X(10); % actual shaft velocity [RPM]
end

% Extract data for plotting
surge = data(:,1);
sway = data(:,2);
yaw_rate = data(:,3);
x_pos = data(:,4);
y_pos = data(:,5);
yaw = data(:,6);
roll_rate = data(:,7);
roll = data(:,8);
rudder = data(:,9);
shaft = data(:,10);

% Display results
figure
subplot(3,1,1)
plot(time, yaw*180/pi, "LineWidth", 1.5);
yline(20, 'r--', 'Desired Course');
grid on;
ylabel("Heading [deg]");

subplot(3,1,2)
plot(time, rudder*180/pi, "LineWidth", 1.5);
grid on;
ylabel("Rudder [deg]");

subplot(3,1,3)
plot(time, yaw_rate*180/pi, "LineWidth", 1.5);
grid on;
ylabel("Yaw rate [deg/s]");
xlabel("Time [s]");

figure
plot(y_pos, x_pos);
title("Trajectory")
axis("equal");
grid on;

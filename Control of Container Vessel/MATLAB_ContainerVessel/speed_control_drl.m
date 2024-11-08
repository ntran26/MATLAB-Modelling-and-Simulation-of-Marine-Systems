% Speed Control Scenario

clear, clc, close all;

load("SpeedControlAgent.mat");
agent = agent1_Trained;

% the state vector: x = [ u v r x y psi p phi delta n ]' for
% a container ship L = 175 m, where
%
% u = surge velocity (m/s)
% v = sway velocity (m/s)
% r = yaw velocity (rad/s)
% x = position in x-direction (m)
% y = position in y-direction (m)
% psi = yaw angle (rad)
% p = roll velocity (rad/s)
% phi = roll angle (rad)
% delta = actual rudder angle (rad)
% n = actual shaft velocity (rpm)
%
% The input vector is :
%
% ui = [ delta_c n_c ]' where
%
% delta_c = commanded rudder angle (rad)
% n_c = commanded shaft velocity (rpm)

% Initial Conditions
u0 = 8;             % Initial surge velocity [m/s]
n_c = 80;           % Initial shaft speed [RPM]
set_u = 10;         % desired speed [m/s]
set_psi = 20;       % desired course
setco = set_psi*pi/180;  % convert to degree

% Initial state vector [u; v; r; x; y; psi; p; phi; delta; n]
X = [u0; 0; 0; 0; 0; 0; 0; 0; 0; n_c];

% Simulation time 
tf = 600;  
dt = 0.1; 
index = 0;

% Simulation
for ii = 0:dt:tf
    index = index + 1;

    % Define observation for the RL agent (e.g., current heading error)
    headingError = wrapToPi(setco - X(6));  % [-pi, pi]
    speedError = abs(set_u - X(1));
    observation = X; 

    % Get the rudder action from the RL agent
    % Convert the observation to a format compatible with the agent
    action = getAction(agent, observation);
    action = action{1};
    
    % Extract rudder angle command from action output
    delta_c = action(1);  
%     delta_c = max(min(delta_c, 10*pi/180), -10*pi/180);

    n_c = action(2);
%     n_c = max(min(n_c, 1), 200);

    uu = [delta_c; n_c];
    [Xdot, U] = container(X, uu);

    % Update state with Euler integration
    X = X + dt*Xdot;

    % Store data
    time(index) = ii;       % time [s]
    data(index,1) = X(1);   % surge velocity [m/s]
    data(index,2) = X(2);   % sway velocity [m/s]
    data(index,3) = X(3);   % yaw rate [rad/s]
    data(index,4) = X(4);   % x-position [m]
    data(index,5) = X(5);   % y-position [m]
    data(index,6) = X(6);   % yaw angle [rad]
    data(index,7) = X(7);   % roll rate [rad/s]
    data(index,8) = X(8);   % roll angle [rad]
    data(index,9) = X(9);   % rudder angle [rad]
    data(index,10) = X(10); % actual shaft velocity [RPM]
end

% Extract data
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
figure;
subplot(4,1,1);
plot(time, surge, 'LineWidth', 1.5);
yline(set_u, 'r--', 'Desired Speed');
grid on;
ylabel("Speed [m/s]");
title("Speed Control with DDPG");

subplot(4,1,2)
plot(time, yaw*180/pi, "LineWidth", 1.5);
yline(set_psi, 'r--', 'Desired Course');
grid on;
ylabel("Heading [deg]");

subplot(4,1,3);
plot(time, shaft, 'LineWidth', 1.5);
grid on;
ylabel("Shaft [m/s]");

subplot(4,1,4)
plot(time, rudder*180/pi, "LineWidth", 1.5);
grid on;
ylabel("Rudder [deg]");
xlabel("Time [s]");

figure
plot(y_pos, x_pos);
title("Trajectory")
xlabel("Y-position [m]");
ylabel("X-position [m]");
axis("equal");
grid on;
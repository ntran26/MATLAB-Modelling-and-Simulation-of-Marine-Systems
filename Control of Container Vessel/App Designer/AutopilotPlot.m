clear, clc, close all;

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

% Define inputs
u0 = 8;             % surge velocity [m/s]
n_ref = 70;
n_c = 70;
setco = 20*pi/180;  % Desired course [degree]

% Initial state vector
X = [u0; 0; 0; 0; 0; 0; 0; 0; 0; n_c];

% Control gains for autopilot
k1 = [2.5; 0.0002; 2];

% Initial values for heading error
eh0 = 0;
ehi = 0;
ehd0 = 0;
eh = [eh0; ehi; ehd0];

% Simulation time
tf = 600;
dt = 0.1;
index = 0;

% Simulation
for ii = 0:dt:tf
    index = index + 1;

    uu = PIDautopilot(k1, eh, n_c);
    [Xdot, U] = container(X, uu);
    
    % Euler's method
    X = X + dt*Xdot;

    % Boundary of yaw angle
    if X(6) >= 2*pi
        X(6) = 2*pi - X(6);
    elseif X(6) <= -2*pi
        X(6) = X(6) + 2*pi;
    else
        X(6) = X(6);
    end
    
    % Calculate heading error
    eh(1) = setco - X(6);

    % Set limits for heading error
    if eh(1) <= -pi
        eh(1) = eh(1) + 2*pi;
    elseif eh(1) >= pi
        eh(1) = eh(1) - 2*pi;
    else
        eh(1) = eh(1);
    end

    % RK2 method
    k11 = dt*eh(1);
    k21 = dt*(eh(1) + k11);
    ehi = ehi + 0.5*(k11 + k21);
    eh(2) = ehi;
    ehd = (eh(1) - ehd0)/dt;
    eh(3) = ehd;

    ehd0 = eh(1);

    % Store data
    time_pid(index) = ii;   % time [s]
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
surge_pid = data(:,1);
sway_pid = data(:,2);
yaw_rate_pid = data(:,3);
x_pos_pid = data(:,4);
y_pos_pid = data(:,5);
yaw_pid = data(:,6);
roll_rate_pid = data(:,7);
roll_pid = data(:,8);
rudder_pid = data(:,9);
shaft_pid = data(:,10);

% Load DDPG agent
load("AutopilotAgent.mat");
agent = agent1_Trained;

% Main loop for deep reinforcement learning agent
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
surge_drl = data(1:6001,1);
sway_drl = data(1:6001,2);
yaw_rate_drl = data(1:6001,3);
x_pos_drl = data(1:6001,4);
y_pos_drl = data(1:6001,5);
yaw_drl = data(1:6001,6);
roll_rate_drl = data(1:6001,7);
roll_drl = data(1:6001,8);
rudder_drl = data(1:6001,9);
shaft_drl = data(1:6001,10);

% Display
figure
subplot(3,1,1)
plot(time_pid, yaw_pid*180/pi, "LineWidth", 1.5);
hold on;
plot(time_pid, yaw_drl*180/pi, "LineWidth", 1.5);
yline(20, 'r--');
grid on;
ylabel("Heading [deg]");
legend("PID", "DDPG", "Desired Course");
title("Autopilot");

subplot(3,1,2)
plot(time_pid, rudder_pid*180/pi, "LineWidth", 1.5);
hold on;
plot(time_pid, rudder_drl*180/pi, "LineWidth", 1.5);
grid on;
ylabel("Rudder [deg]");

subplot(3,1,3)
plot(time_pid,yaw_rate_pid*180/pi, "LineWidth", 1.5);
hold on;
plot(time_pid,yaw_rate_drl*180/pi, "LineWidth", 1.5);
grid on;
ylabel("Yaw rate [deg/s]");
xlabel("Time [s]");

figure
plot(y_pos_pid, x_pos_pid);
hold on;
plot(y_pos_drl, x_pos_drl);
title("Autopilot Trajectory")
xlabel("Y-position [m]");
ylabel("X-position [m]");
axis("equal");
grid on;

function ui = PIDautopilot(k,ee,n_c)
    % k(1) = kp, k(2) = ki, k(3) = kd
    % delta_c = commanded rudder angle
    % n_c = commanded shaft velocity

    delta_c = k(1)*ee(1) + k(2)*ee(2) + k(3)*ee(3);

    if delta_c <= -10*pi/180
        delta_c = -10*pi/180;
    elseif delta_c >= 10*pi/180
        delta_c = 10*pi/180;
    else
        delta_c = delta_c;
    end

    ui = [delta_c
          n_c];
end


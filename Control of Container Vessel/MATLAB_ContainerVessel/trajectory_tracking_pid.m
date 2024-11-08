% Waypoints
% wpt.pos.x = [0, 500, 800, 700, 1000];
% wpt.pos.y = [0, 400, 650, 900, 1000];

wpt.pos.x = [0, 1000, 2000, 3000];
wpt.pos.y = [0, 1500, 1500, 1000];

tolerance = 20; % Tolerance for waypoint switching in meters

% Initial settings
u0 = 8;             % Initial surge velocity [m/s]
n_c = 80;           % Initial shaft speed [RPM]
set_u = 9;         % Desired speed [m/s]
lookahead_dist = 100; % Lookahead distance for LOS guidance
current_wp = 2;     % Start aiming for the second waypoint

% State and error initialization
X = [u0; 0; 0; 0; 0; 0; 0; 0; 0; n_c];

% Control gains for speed controller
k_head = [2.5; 0.0002; 2];
% k_speed = [15; 0.0002; 2.5]; 
k_speed = [196; 5.9; 414.38];

% Inital values for heading error
eh0 = 0;
ehi = 0;
ehd0 = 0;
eh = [eh0; ehi; ehd0];

% Initial values for speed error
es0 = 0;
esi = 0;                        
esd0 = 0;                       
es = [es0; esi; esd0];

% Simulation time
tf = 600;
dt = 0.1;
index = 0;

% Main simulation loop
for ii = 0:dt:tf
    index = index + 1;

    % Calculate distance to the current waypoint
    dx = wpt.pos.x(current_wp) - X(4);
    dy = wpt.pos.y(current_wp) - X(5);
    dist_to_wp = sqrt(dx^2 + dy^2);

    % Check if close enough to switch to the next waypoint
    if dist_to_wp < tolerance && current_wp < length(wpt.pos.x)
        current_wp = current_wp + 1;
        dx = wpt.pos.x(current_wp) - X(4);
        dy = wpt.pos.y(current_wp) - X(5);
    end

    % LOS guidance to determine desired heading
    desired_heading = atan2(dy, dx);
    psi_LOS = atan2(dy, lookahead_dist); % LOS angle calculation
    eh(1) = psi_LOS - X(6); % Heading error for PID control

    % PID heading control
    u_head = PIDautopilot(k_head, eh);

    % PID speed control
    u_speed = PIDspeedControl(k_speed, es, X(10));

    % Apply control inputs to the vessel model
    uu = [u_head; u_speed];
    [Xdot, U] = container(X, uu);

    % Update state with Euler integration
    X = X + dt*Xdot;

    % Calculate and update integral and derivative terms for heading PID
    eh(2) = eh(2) + eh(1) * dt;
    eh(3) = (eh(1) - ehd0) / dt;
    ehd0 = eh(1);

    % Calculate speed error
    es(1) = set_u - X(1);
    es(2) = es(2) + es(1) * dt;
    es(3) = (es(1) - esd0) / dt;
    esd0 = es(1);

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

% Plot results
figure;
subplot(4,1,1); plot(time, data(:,1), 'LineWidth', 1.5); hold on;
yline(set_u, 'r--', 'Desired Speed'); ylabel('Speed [m/s]'); grid on;
subplot(4,1,2); plot(time, data(:,6)*180/pi, 'LineWidth', 1.5); hold on;
yline(rad2deg(desired_heading), 'r--', 'Desired Heading'); ylabel('Heading [deg]'); grid on;
subplot(4,1,3); plot(time, data(:,10), 'LineWidth', 1.5); ylabel('Shaft [RPM]'); grid on;
subplot(4,1,4); plot(time, data(:,9)*180/pi, 'LineWidth', 1.5); ylabel('Rudder [deg]'); grid on;
xlabel('Time [s]');

% Plot trajectory with waypoints marked
figure; 
% Vessel's path
plot(y_pos,x_pos, 'b-', 'LineWidth', 1.5);
hold on;
% Desired path
plot(wpt.pos.y, wpt.pos.x, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(wpt.pos.y, wpt.pos.x, 'r--', 'LineWidth', 1.5);
title('Path Following Trajectory');
xlabel('Y-position [m]');
ylabel('X-position [m]');
legend('Vessel Trajectory', 'Waypoints');
axis equal;
grid on;

function uu = PIDautopilot(k,ee)
    delta_c = k(1)*ee(1) + k(2)*ee(2) + k(3)*ee(3);
    delta_c = max(min(delta_c, 10*pi/180), -10*pi/180); % Limit rudder angle
    uu = delta_c;
end

function ui = PIDspeedControl(k, ee, n_current)
    delta_n = k(1)*ee(1) + k(2)*ee(2) + k(3)*ee(3);
    n_c = max(min(n_current + delta_n, 200), 1); % Limit RPM
    ui = n_c;
end

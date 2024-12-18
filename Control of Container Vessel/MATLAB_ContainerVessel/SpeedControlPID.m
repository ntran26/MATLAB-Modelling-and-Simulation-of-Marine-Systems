% Speed Control Scenario

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

% Initial Conditions
u0 = 8;             % Initial surge velocity [m/s]
n_c = 80;           % Initial shaft speed [RPM]
set_u = 10;         % desired speed [m/s]
set_psi = 20;       % desired course
setco = set_psi*pi/180;  % convert to degree

% Initial state vector [u; v; r; x; y; psi; p; phi; delta; n]
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

% Simulation
for ii = 0:dt:tf
    index = index + 1;

    % Call the PID heading controller
    u_head = PIDautopilot(k_head, eh);

    % Call the PID speed controller
    u_speed = PIDspeedControl(k_speed, es, X(10));

    uu = [u_head; u_speed];
    [Xdot, U] = container(X, uu);

    % Update state with Euler integration
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
    
    % RK2 method
    k11 = dt*eh(1);
    k21 = dt*(eh(1) + k11);
    ehi = ehi + 0.5*(k11 + k21);
    eh(2) = ehi;
    ehd = (eh(1) - ehd0)/dt;
    eh(3) = ehd;
    ehd0 = eh(1);
    
    % Calculate speed error
    es(1) = set_u - X(1);  

    % Update error terms for PID, use RK2 integrator
    k11 = dt * es(1);
    k21 = dt * (es(1) + k11);
    esi = esi + 0.5 * (k11 + k21);  % Integral term
    es(2) = esi;
    esd = (es(1) - esd0) / dt;      % Derivative term
    es(3) = esd;
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

% Display results
figure;
subplot(4,1,1);
plot(time, surge, 'LineWidth', 1.5);
yline(set_u, 'r--', 'Desired Speed');
grid on;
ylabel("Speed [m/s]");
title("Speed Control with PID");

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

function uu = PIDautopilot(k,ee)
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

    uu = delta_c;
end

function ui = PIDspeedControl(k, ee, n_current)
    % PID controller for speed control
    % k(1) = Kp, k(2) = Ki, k(3) = Kd
    % delta_c = commanded change in RPM
    % n_c = current commanded RPM

    delta_n = k(1)*ee(1) + k(2)*ee(2) + k(3)*ee(3);
    n_c = n_current + delta_n;

    if n_c >= 200
        n_c = 200;
    elseif n_c <= 1
        n_c = 1;
    else
        n_c = n_c;
    end

    ui = n_c;
end
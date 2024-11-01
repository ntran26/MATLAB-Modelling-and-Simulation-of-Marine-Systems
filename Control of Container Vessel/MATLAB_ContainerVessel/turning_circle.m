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
u0 = 8;     % surge velocity [m/s]
u_c = [20*pi/180
       80];     % commanded rudder [deg]
X = [u0; 0; 0; 0; 0; 0; 0; 0; 0; 80];   % initial state vector

tf = 600;
dt = 0.1;
index = 0;

% Simulation
for ii = 0:dt:tf
    index = index + 1;
    [Xdot, U] = container(X, u_c);
    
    % Euler's method
    X = X + dt*Xdot;

    % Boundary of yaw angle
    if X(6) > 2*pi
        X(6) = X(6) - 2*pi;
    elseif X(6) <= 0
        X(6) = X(6) + 2*pi;
    else
        X(6) = X(6);
    end

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

% Display
figure
subplot(4,1,1)
plot(time, rudder*180/pi);
grid on;
ylabel("Rudder angle [deg]");

subplot(4,1,2)
plot(time, yaw_rate*180/pi);
grid on;
ylabel("Yaw rate [deg/s]");

subplot(4,1,3)
plot(time,yaw*180/pi);
grid on;
ylabel("Yaw angle [deg]");

subplot(4,1,4)
plot(time, roll*180/pi);
grid on;
ylabel("Roll angle [deg]");
xlabel("Time [s]");

figure
plot(y_pos, x_pos);
axis("equal");
grid on;
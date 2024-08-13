clear;

% Initial condition
x_a0 = [1;1]*pi/180;         % convert to radian
x_b0 = [-5;2]*pi/180;
x_c0 = [5;2]*pi/180;

% Time span
tspan = [0 20];         % 0 to 10 seconds

% Application of ode45
[t_a, y_a] = ode45(@pendulum, tspan, x_a0);
[t_b, y_b] = ode45(@pendulum, tspan, x_b0);
[t_c, y_c] = ode45(@pendulum, tspan, x_c0);

% Plotting
figure
subplot(3,1,1)
plot(t_a, y_a, '--*');
% xlabel('t [s], y(1) & y(2)');
ylabel('y(1) & y(2)');
legend('y1 [rad], y2 [rad/s]');

subplot(3,1,2)
plot(t_b, y_b, '--*');
% xlabel('t [s], y(1) & y(2)');
ylabel('y(1) & y(2)');
legend('y1 [rad], y2 [rad/s]');

subplot(3,1,3)
plot(t_c, y_c, '--*');
xlabel('t [s], y(1) & y(2)');
ylabel('y(1) & y(2)');
legend('y1 [rad], y2 [rad/s]');

% Initial condition
x10 = [-1; 2];

% Solving
[t1, y1] = ode45(@activity1_eq1, tspan, x10);

% Calculate the exact solution
t = 0:0.01:20;
y_ex1 = 5/2 - 5*exp(-t) + 3/2*exp(-2*t);

% Plotting
figure
plot(t1,y1(:,1), 'b', t, y_ex1, '--r');
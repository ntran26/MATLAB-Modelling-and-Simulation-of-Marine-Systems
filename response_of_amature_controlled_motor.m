clear, clc, close all;

u = [10; 0];
ts = [0 2];
index = 0;
for ii = 0:0.00001:ts(2)
    index = index+1;
    t1(index) = ii;
    u1(index) = u(1);
end
x0 = [0; 0; 0];
[t,x] = ode45(@ex47, ts, x0, [], u);
plot(t, x(:,2),'r', t1, u1,'b');
axis([0, ts(2), 0, 200]);
xlabel('Time (s)')
ylabel('Response y')
title('Response of motor system')
legend('y','u')

function xdot = ex47(t,x,u)
    La = 56e-5;     % armature inductance
    Ra = 1.35;      % armature resistance
    J = 1.9e-3;     % moment of inertia
    Kb = 0.1;       % back-emf constant
    K = 0.1;        % motor constant
    b = 0.000792;   % viscous friction coefficient

    xdot = [x(2)
            -b/J*x(2)+K/J*x(3)
            -K/La*x(2)-Ra/La*x(3)+1/La*u(1)];

%     A = [0 1 0
%         0 -b/J K/J
%         0 -K/La -Ra/La];
%     B = [0; 0; 1/La];
%     xdot = A*x + B*u(1);
end
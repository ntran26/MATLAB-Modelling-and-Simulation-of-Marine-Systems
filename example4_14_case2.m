clear, clc, close all;

%{
T(s) = Vo(s)/Vi(s) = K/(Ts+1)
%}

ts = [0 2];
omega1 = 2*pi*0.5;
omega2 = 2*pi*10;
A1 = 2;
A2 = -3;
Rc = 0.02;
u0 = 5;
x = [0];
h = 0.0001;
i = 0;

for t = 0:h:ts(2)
    i = i + 1;
    u = u0 + A1*sin(omega1*t) + A2*sin(omega2*t);

    xdot = [-1/Rc*x(1) + 1/Rc*u(1)];
    x = x + xdot*h;
    
    data(i, 1) = t;
    data(i, 2) = u(1);
    data(i, 3) = x;
end

plot(data(:,1), data(:,2), 'r', data(:,1), data(:,3), 'b');
% axis([0, ts(2), -10, 10]);
xlabel('Time (secs)')
ylabel('Response y')
title('Response of motor system')
legend('u','y')
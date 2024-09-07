clear, clc, close all;

%{
T(s) = Vo(s)/Vi(s) = K/(Ts+1)
%}

ts = [0 2];
omega = 15;
Rc = 0.02;
x = [0];
h = 0.0001;
i = 0;

for t = 0:h:ts(2)
    i = i + 1;
    u = 2*sin(omega*t);

    xdot = [-1/Rc*x(1) + 1/Rc*u(1)];
    x = x + xdot*h;
    
    data(i, 1) = t;
    data(i, 2) = u(1);
    data(i, 3) = x;
end

plot(data(:,1), data(:,2), 'r', data(:,1), data(:,3), 'b');
axis([0, ts(2), -10, 10]);
xlabel('Time (s)')
ylabel('Response y')
title('Response of motor system')
legend('u', 'y')


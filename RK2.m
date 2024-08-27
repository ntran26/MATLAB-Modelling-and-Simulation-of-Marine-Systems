clear, clc, close all;

ts = 0;
tf = 2;

% Analytical solution
t = ts:0.01:tf;
yt = 1 - exp(-t);

plot(t, yt, 'k'), hold on

% Numerical solution
h = 0.1;
y(1) = 0;
x(1) = 0;

for k = 1:tf*(1/h)
    x(k+1) = (1-h)*x(k) + h;
    y(k+1) = y(k) - (h/2)*(y(k) + x(k+1)) + h; 
end

tn = ts:h:tf;
plot(tn, x, 'r', tn, x, 'ro')
plot(tn, y, 'b', tn, y, 'kd')


ts = 0; tf = 2;
t = ts:0.01:tf; yt = 1-exp(-t); plot(t, yt, 'k'), hold on
% Numerical solution
h = 0.5; y(1,:) = 0; % Step size and initial value
f = inline('-y + 1','t','y'); % Construct a function
                              % ODE: dy/dt + y = 1
for k = 1:tf*(1/h)
    k1 = h*feval(f, t(k),y(k,:)); k1 = k1(:)';
    k2 = h*feval(f, t(k)+h/2, y(k,:) + k1/2); k2 = k2(:)';
    k3 = h*feval(f, t(k)+h/2, y(k,:) + k2/2); k3 = k3(:)';
    k4 = h*feval(f, t(k)+h, y(k,:) + k3); k4 = k4(:)';
    y(k + 1,:) = y(k,:) + (k1 + 2*(k2 + k3) + k4)/6;
end
tn=[ts:h:tf]; % Make time span for plotting result
plot(tn, y, 'm', tn, y, 'm*')

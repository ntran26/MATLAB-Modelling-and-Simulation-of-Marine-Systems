clc, clear, close all;

% Start and final time
ts = 0;
tf = 2;

% Analytical solution
t = ts:0.01:tf;     % time span 
yt = 1 - exp(-t);     % true analytical solution
line1 = plot(t, yt, 'k'); hold on

% Numerical solution
h1 = 0.25;   % step size
h2 = 0.5;
h3 = 1;
y1(1) = 0;   % initial value
y2(1) = 0;
y3(1) = 0;

% Step size = 0.25
for k1 = 1:tf*(1/h1)
    y1(k1 + 1) = (1 - h1)*y1(k1) + h1;   % difference equation
end

% Step size = 0.5
for k2 = 1:tf*(1/h2)
    y2(k2 + 1) = (1 - h2)*y2(k2) + h2;   
end

% Step size = 1
for k3 = 1:tf*(1/h3)
    y3(k3 + 1) = (1 - h3)*y3(k3) + h3;   % difference equation
end

% Time span for plotting result
t1 = ts:h1:tf;
t2 = ts:h2:tf;
t3 = ts:h3:tf;

% Adding plots to compare
line2 = plot(t1, y1, 'b', 'DisplayName', 'Euler 0.25'); hold on
plot(t1, y1, 'bo'), hold on
line3 = plot(t2, y2, 'g', 'DisplayName', 'Euler 0.5'); hold on
plot(t2, y2, 'go'), hold on
line4 = plot(t3, y3, 'r', 'DisplayName', 'Euler 1'); hold on
plot(t3, y3, 'ro')
legend([line1 line2 line3 line4], ...
    {'Analytical','Euler 0.25', 'Euler 0.5', 'Euler 1'}, ...
    'Location','northwest','NumColumns',1);


% Example 4


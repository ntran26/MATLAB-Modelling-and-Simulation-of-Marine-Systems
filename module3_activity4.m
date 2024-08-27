% RK2 method and exact solution

clear, clc, close all;

x = [-1; 2];    % initial condition
h = 0.01;       % step size
tf = 10;        % final time

i = 0;
for ii = 0:h:tf
    i = i+1;
    u = 1;

    k1 = h*activity4(x,u,ii);           % time [s]
    k2 = h*activity4(x+k1,u,ii);        % y
    x = x + 0.5*(k1+k2);                % y dot
    y=2.5-5*exp(-ii)+1.5*exp(-2*ii);    % exact solution

    data(i,1)=ii;
    data(i,2)=x(1);
    data(i,3)=x(2);
    data(i,4)=y;
end

plot(data(:,1),data(:,2), 'b', data(:,1),data(:,4), 'r')


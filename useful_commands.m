% Create matrix
A = [1, 2, 3; 4, 5, 6; 7, 8, 9]
B = [1, 4, 7; 5, 2, 8; 9, 6, 3]

% Matrix dot product
C = A*B

% Inverted matrix
D = inv(B)

% Eigen value
eigen = eig(A)

% Use function
x = 3;
recip = reciprocal(x)

y = positive(-1)
y = positive(0)

% Plotting
ome = 2;
ss = 0.01;
t = 0:ss:10;        % generate time vector
y = exp(-t);        % function y
z = 2*sin(ome*t);   % function x

subplot(2,1,1);
plot(t,y);
grid on;
title('Plotting of two functions');
xlabel('Time [s]');
ylabel('y[-]');

subplot(2,1,2);
plot(t,z);
grid on;
xlabel('Time [s]');
ylabel('z[-]');

% Function that returns the reciprocal of the input: f(x) = 1/x
function a = reciprocal(x)
a = 1/x;
end

% Function that swaps 2 input variables
function [a, b] = swap(x, y)
a = y;
b = x;
end

% if else statement
function y = positive(x)
    if x >= 0
        y = 1;
    else
        y = 0;
    end
end

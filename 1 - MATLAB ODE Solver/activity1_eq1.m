function [x_dot] = activity1_eq1(t,x)
    u = 1;      % input

    % state space model
    x_dot = [x(2)
             -2*x(1) - 3*x(2) + 5*u];
end
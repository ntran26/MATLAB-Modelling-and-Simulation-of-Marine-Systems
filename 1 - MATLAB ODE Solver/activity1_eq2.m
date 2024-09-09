function [x_dot] = activity1_eq2(t,x)
    u = 1;      % input

    % state space model
    x_dot = [x(2)
             -5*x(1) - 2*x(2) + 5*u];

end
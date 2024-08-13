% Function for pendulum system without exiting force
function x_dot = pedulum(t,x)
    
    g = 9.81;   % gravity
    L = 10;     % length of pendulum

    x_dot = [x(2)
             -g/L*sin(x(1))];
end
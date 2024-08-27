function x_dot = activity4(x, u, t)
    % System parameters
    A = [0 1
        -2 -3];
    B = [0; 5];
    
    x_dot = A*x + B*u;
end




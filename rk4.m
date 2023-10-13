function [time_steps, X] = rk4(x_0, u_bar, T, N)
    % x_0: initial state vector
    % u_bar: N-step input sequence
    % T: time step for the discrete time approx
    % N: number of steps in the prediction horizon
    
    x_i = x_0;
    X = [x_i];
    time_steps = [0];
    for i = 1:N
        K1 = f(x_i, u_bar(i));
        K2 = f(x_i + K1*T/2, u_bar(i));
        K3 = f(x_i + K2*T/2, u_bar(i));
        K4 = f(x_i + K3*T, u_bar(i));
        
        K = (K1 + 2*K2 + 2*K3 + K4)/6;
        x_i = x_i + K*T;
        X(end+1) = x_i;
        time_steps(end+1) = T*i;
    end
end

function vdot = f(v, u)
    % define constants
    m = 1000.0;
    p = 1.2;
    Cd = 0.4;
    A = 5.0;
    Crr = 0.01;
    g = 9.8;

    % dynamics equation
    vdot = (u - 0.5*p*Cd*A*v^2 - Crr*m*g)/m;
end
%% Time Discretizer Function
function [x_i1] = rk4_symbolic(eom)
    % Define the states
    syms x y z u v w phi theta psy p q r 

    % Define the inputs
    syms w1 w2 w3 w4

    % Define the time interval
    syms dt

    % Define the vectors
    x_i = [x, y, z, u, v, w, phi, theta, psy, p, q, r];
    u_i = [w1, w2, w3, w4];

    % Compute the approximations
    [x_out, y_out, z_out, u_out, v_out, w_out, phi_out, theta_out, psy_out, p_out, q_out, r_out, w1_out, w2_out, w3_out, w4_out] = parser(x_i, u_i);
    K1 = eom(x_out, y_out, z_out, u_out, v_out, w_out, phi_out, theta_out, psy_out, p_out, q_out, r_out, w1_out, w2_out, w3_out, w4_out);

    [x_out, y_out, z_out, u_out, v_out, w_out, phi_out, theta_out, psy_out, p_out, q_out, r_out, w1_out, w2_out, w3_out, w4_out] = parser(x_i + K1*dt/2, u_i);
    K2 = eom(x_out, y_out, z_out, u_out, v_out, w_out, phi_out, theta_out, psy_out, p_out, q_out, r_out, w1_out, w2_out, w3_out, w4_out);

    [x_out, y_out, z_out, u_out, v_out, w_out, phi_out, theta_out, psy_out, p_out, q_out, r_out, w1_out, w2_out, w3_out, w4_out] = parser(x_i + K2*dt/2, u_i);
    K3 = eom(x_out, y_out, z_out, u_out, v_out, w_out, phi_out, theta_out, psy_out, p_out, q_out, r_out, w1_out, w2_out, w3_out, w4_out);

    [x_out, y_out, z_out, u_out, v_out, w_out, phi_out, theta_out, psy_out, p_out, q_out, r_out, w1_out, w2_out, w3_out, w4_out] = parser(x_i + K3*dt, u_i);
    K4 = eom(x_out, y_out, z_out, u_out, v_out, w_out, phi_out, theta_out, psy_out, p_out, q_out, r_out, w1_out, w2_out, w3_out, w4_out);
    
    % Compute the weigthed average 
    K = (K1 + 2*K2 + 2*K3 + K4)/6;

    % Compute the symbolic equation for the i+1 state
    x_i1 = x_i + K*dt;
end

% function vdot = f(v, u)
%     % define constants
%     m = 1000.0;
%     p = 1.2;
%     Cd = 0.4;
%     A = 5.0;
%     Crr = 0.01;
%     g = 9.8;
% 
%     % dynamics equation
%     vdot = (u - 0.5*p*Cd*A*v^2 - Crr*m*g)/m;
% end
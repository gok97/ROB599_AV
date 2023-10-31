%% Time Discretizer Function
function [next_state] = rk4_symbolic(state, input, dt, constants)

    % Compute the approximations
    % [x_out, y_out, z_out, u_out, v_out, w_out, phi_out, theta_out, psy_out, p_out, q_out, r_out, w1_out, w2_out, w3_out, w4_out] = parser(x_i, u_i);
    % K1 = eom(x_out, y_out, z_out, u_out, v_out, w_out, phi_out, theta_out, psy_out, p_out, q_out, r_out, w1_out, w2_out, w3_out, w4_out);
    [tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3] = EquationsOfMotion(state, input, constants, false, false);
    K1 = [tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3];

    % [x_out, y_out, z_out, u_out, v_out, w_out, phi_out, theta_out, psy_out, p_out, q_out, r_out, w1_out, w2_out, w3_out, w4_out] = parser(x_i + K1*dt/2, u_i);
    % K2 = eom(x_out, y_out, z_out, u_out, v_out, w_out, phi_out, theta_out, psy_out, p_out, q_out, r_out, w1_out, w2_out, w3_out, w4_out);
    [tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3] = EquationsOfMotion(state + K1*dt/2, input, constants, false, false);
    K2 = [tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3];

    % [x_out, y_out, z_out, u_out, v_out, w_out, phi_out, theta_out, psy_out, p_out, q_out, r_out, w1_out, w2_out, w3_out, w4_out] = parser(x_i + K2*dt/2, u_i);
    % K3 = eom(x_out, y_out, z_out, u_out, v_out, w_out, phi_out, theta_out, psy_out, p_out, q_out, r_out, w1_out, w2_out, w3_out, w4_out);
    [tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3] = EquationsOfMotion(state + K2*dt/2, input, constants, false, false);
    K3 = [tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3];

    % [x_out, y_out, z_out, u_out, v_out, w_out, phi_out, theta_out, psy_out, p_out, q_out, r_out, w1_out, w2_out, w3_out, w4_out] = parser(x_i + K3*dt, u_i);
    % K4 = eom(x_out, y_out, z_out, u_out, v_out, w_out, phi_out, theta_out, psy_out, p_out, q_out, r_out, w1_out, w2_out, w3_out, w4_out);
    [tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3] = EquationsOfMotion(state + K3*dt, input, constants, false, false);
    K4 = [tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3];
    
    % Compute the weigthed average 
    K = (K1 + 2*K2 + 2*K3 + K4)/6;

    % Compute the symbolic equation for the i+1 state
    next_state = state + K*dt;
end

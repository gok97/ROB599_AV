%% Time Discretizer Function
function [next_state] = rk4(state, input, dt, constants, t, mass_type, symbolic)

    % Compute the approximations
    [tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3] = EquationsOfMotion(state, input, constants, t, mass_type, symbolic, false);
    K1 = [tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3];

    [tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3] = EquationsOfMotion(state + K1*dt/2, input, constants, t, mass_type, symbolic, false);
    K2 = [tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3];

    [tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3] = EquationsOfMotion(state + K2*dt/2, input, constants, t, mass_type, symbolic, false);
    K3 = [tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3];

    [tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3] = EquationsOfMotion(state + K3*dt, input, constants, t, mass_type, symbolic, false);
    K4 = [tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3];
    
    % Compute the weigthed average 
    K = (K1 + 2*K2 + 2*K3 + K4)/6;

    % Compute the symbolic equation for the i+1 state
    next_state = state + K*dt;
end

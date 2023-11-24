function [A, B] = discretize_and_compute_jacobians(state, input, dt, eom_params, XU0)
    % discretize continuous system
    next_state = rk4(state, input, dt, eom_params);
    tk1 = next_state(1);
    tk2 = next_state(2);
    tk3 = next_state(3);
    td1 = next_state(4);
    td2 = next_state(5);
    td3 = next_state(6);
    rk1 = next_state(7);
    rk2 = next_state(8);
    rk3 = next_state(9);
    rd1 = next_state(10);
    rd2 = next_state(11);
    rd3 = next_state(12);

    % Compute the jacobians
    [A, B] = Linearizer(tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3, XU0, false);
    A = double(A);
    B = double(B);
end
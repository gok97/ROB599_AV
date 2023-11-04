function [Xsim, Usim] = sim_linear_mpc(A, B, mpc_params)
    % extract necessary parameters
    X0 = mpc_params{4};
    dt = mpc_params{10};
    eom_params = mpc_params{11};
    % Execute the simulation
    N_sim = 100;
    Xsim = [X0];
    Usim = [];
    for i = 1:N_sim
        Xsim(i, :) = X0;
    end
    for i = 1:N_sim-1
        Usim(i, :) = [0, 0, 0, 0];
    end
    Uprev = [0, 0, 0, 0];
    for i = 1:(N_sim-1)
        fprintf("simulation iteration: %d\n", i);
        % update current time
        current_time = dt*i;
        eom_params{2} = current_time;
        mpc_params{11} = eom_params;
        % get control input from linear MPC
        Usim(i, :) = DroneMPC(A, B, mpc_params, Xsim(i, :), Uprev, i);
        Uprev = Usim(i, :);
        % Calculate the next step of the simulation
        Xsim(i+1, :) = rk4(Xsim(i, :), Usim(i, :), dt, eom_params);
    end
end



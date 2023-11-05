function [Xsim, Usim] = sim_linear_mpc(XU0, mpc_params)
    % Define the states
    syms x y z u v w phi theta psy p q r
    state = [x y z u v w phi theta psy p q r];

    % Define the inputs
    syms w1 w2 w3 w4
    input = [w1 w2 w3 w4];

    % extract necessary parameters
    X0 = mpc_params{4};
    dt = mpc_params{10};
    eom_params = mpc_params{11};
    mass = eom_params{3};
    wind = eom_params{4};
    
    % extract mass constants
    m_prev = mass(1, 1);
    Ix_prev = mass(1, 2);
    Iy_prev = mass(1, 3);
    Iz_prev = mass(1, 4);
    
    % extract wind constants
    xdot_w_prev = wind(1, 1);
    ydot_w_prev = wind(1, 2);
    zdot_w_prev = wind(1, 3);

    % Execute the simulation
    Xref = mpc_params{6};
    N_sim = length(Xref);
    Xsim = [X0];
    Usim = [];
    for i = 1:N_sim
        Xsim(i, :) = X0;
    end
    for i = 1:N_sim-1
        Usim(i, :) = [0, 0, 0, 0];
    end

    Uprev = [0, 0, 0, 0];

    % Compute initial jacobians
    [A, B] = discretize_and_compute_jacobians(state, input, dt, eom_params, XU0);

    for i = 1:(N_sim-1)
        fprintf("simulation iteration: %d\n", i);
        % update current time
        current_index = i;
        eom_params{2} = current_index;
        mpc_params{11} = eom_params;
        
        % Compute jacobians if necessary
         
        % [A, B] = discretize_and_compute_jacobians(state, input, dt, eom_params, XU0);
        [A, B] = compute_jacobians_if_necessary(A, B, state, input, dt, eom_params, XU0, m_prev, xdot_w_prev, ydot_w_prev, mass, wind, current_index);

        % get control input from linear MPC
        Usim(i, :) = DroneMPC(A, B, mpc_params, Xsim(i, :), Uprev, i);
        % Calculate the next step of the simulation
        Xsim(i+1, :) = rk4(Xsim(i, :), Usim(i, :), dt, eom_params);

        % Update variables
        Uprev = Usim(i, :);
        m_prev = mass(current_index, 1);
        xdot_w_prev = wind(current_index, 1);
        ydot_w_prev = wind(current_index, 2);
    end
end

function [A, B] = compute_jacobians_if_necessary(A_old, B_old, state, input, dt, eom_params, XU0, m_prev, xdot_w_prev, ydot_w_prev, mass, wind, current_index)
    % extract mass constants
    m_curr = mass(current_index, 1);
    
    % extract wind constants
    xdot_w_curr = wind(current_index, 1);
    ydot_w_curr = wind(current_index, 2);

    if (m_prev == m_curr) && (xdot_w_prev == xdot_w_curr) && (ydot_w_prev == ydot_w_curr)
        A = A_old;
        B = B_old;
    else
        [A, B] = discretize_and_compute_jacobians(state, input, dt, eom_params, XU0);
    end


end

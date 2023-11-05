%% Define the function that performs Model Predictive Control for the MPC
function control_input = DroneMPC(A, B, parameters, initial_conditions, Uprev, time_index)
    % Begin the cvx problem
    cvx_begin

        % Extract the needed variables from the parameters cell array:
        horizon = parameters{1};
        Q = parameters{2};
        R = parameters{3};
        Xbar =  parameters{4};
        Ubar =  parameters{5};
        Xref =  parameters{6}(time_index:(time_index + horizon - 1), :);
        Xref = create_valid_horizon(Xref, horizon);
        Uref =  parameters{7}(time_index:(time_index + horizon - 2), :);
        Uref = create_valid_horizon(Uref, horizon);
        dt = parameters{10};
        eom_params = parameters{11};

        % Define the delta_x and delta_u as cvx variables
         variable delta_X(horizon, parameters{8});
         variable delta_U(horizon - 1, parameters{9});
        
        % Construct the cost function
        cost = 0;
        for i = 1:horizon
            xi = Xbar + delta_X(i, :);
            cost = cost + 0.5*quad_form(xi - Xref(i, :), Q);
        end
        for i = 1:(horizon - 1)
            ui = Ubar + delta_U(i, :);
            cost = cost+ 0.5*quad_form(ui - Uref(i, :), R);
        end
    
        % Define the problem type
        minimize(cost);
        
        subject to
        % Define the initial condition constraint
        Xbar + delta_X(1, :) == initial_conditions;
    
        % Define the dynamics and control constraints
        for i = 1:(horizon-1)
            % update mpc time
            mpc_index = time_index + i;
            eom_params{2} = mpc_index;
            % dynamics constraints
            Xbar + delta_X(i+1, :) == rk4(Xbar, Ubar, dt, eom_params)  + delta_X(i, :)*A' + delta_U(i, :)*B';
            % control input constraints
            Ubar + delta_U(i, :) <= Ubar(1)*5;
            Ubar + delta_U(i, :) >= 0.0;
            Ubar + delta_U(i, :) - Uprev >= -Ubar(1)*5;
            Ubar + delta_U(i, :) - Uprev <= (Ubar(1)*5) / 2;
            Uprev = Ubar + delta_U(i, :);
        end

    cvx_end
    
    % Compute the desired output
    control_input = Ubar + delta_U(1, :);
end

function new_ref = create_valid_horizon(ref, horizon)
    if length(ref) == horizon
        new_ref = ref;
    else
        last_val = ref(1, :);
        new_ref = [ref];
        for i = 1:(length(ref)-horizon)
            new_ref = [new_ref; last_val];
        end
    end
end
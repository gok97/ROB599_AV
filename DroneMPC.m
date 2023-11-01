%% Define the function that performs Model Predictive Control for the MPC
function control_input = DroneMPC(A, B, parameters, initial_conditions, time_index)
    % Begin the cvx problem
    cvx_begin

        % Extract the needed variables from the parameters cell array:
        horizon = parameters{1};
        Q = parameters{2};
        R = parameters{3};
        Xbar =  parameters{4};
        Ubar =  parameters{5};
        Xref =  parameters{6}(time_index:(time_index + horizon - 1), :);
        Uref =  parameters{7}(time_index:(time_index + horizon - 2), :);
        dt = parameters{10};
        K = parameters{11};
        mass_type = parameters{12};

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
    
        % Define the dynamic constraints
        for i = 1:(horizon-1)
            time = (time_index + i) * dt;
            Xbar + delta_X(i+1, :) == rk4(Xbar, Ubar, dt, K, time, mass_type, false)  + delta_X(i, :)*A' + delta_U(i, :)*B';
        end

    cvx_end
    
    % Compute the desired output
    control_input = Ubar(1) + delta_U(1, :);
end
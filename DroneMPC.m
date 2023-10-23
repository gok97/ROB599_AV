%% Define the function that performs Model Predictive Control for the MPC
function control_input = DroneMPC(A, B, eom_list, parameters, initial_conditions, time_index)
    
    % Begin the cvx problem
    cvx_begin

        % Extract the needed variables from the parameters cell array:
        % parameters = [horizon, Q, R, Xbar, Ubar, Xref, Uref, nx, nu, dt]
        horizon = parameters{1};
        Q = parameters{2};
        R = parameters{3};
        Xbar =  parameters{4};
        Ubar =  parameters{5};
        Xref =  parameters{6}(time_index:(time_index + horizon - 1));
        Uref =  parameters{7}(time_index:(time_index + horizon - 2));
        dt = parameters{10};

        % Define the delta_x and delta_u as cvx variables
         variable delta_X(horizon, parameters{8});
         variable delta_U(horizon - 1, parameters{9});
        
        % Construct the cost function
        cost = 0;
        for i = 1:horizon
            xi = Xbar + delta_X(i, :);
            cost = cost + 0.5*quad_form(xi - Xref(i), Q);
        end
        for i = 1:(horizon - 1)
            ui = Ubar + delta_U(i, :);
            cost = cost+ 0.5*quad_form(ui - Uref(i), R);
        end
    
        % Define the problem type
        minimize(cost);
    
        % Define the initial condition constraint
        Xbar + delta_X(1, :) == initial_conditions;
    
        % Define the dynamic constarints
        for i = 1:(horizon-1)
           [x_out, y_out, z_out, u_out, v_out, w_out, phi_out, theta_out, psy_out, p_out, q_out, r_out, w1_out, w2_out, w3_out, w4_out] = parser(Xbar, Ubar);
           A = double(A);
           B = double(B);
           Xbar + delta_X(i+1, :) == eom_list(x_out, y_out, z_out, u_out, v_out, w_out, phi_out, theta_out, psy_out, p_out, q_out, r_out, w1_out, w2_out, w3_out, w4_out) + delta_X(i, :)*A' + delta_U(i, :)*B';
        end
    
        %cvx.solve!(prob, ECOS.Optimizer; silent_solver = true)

    cvx_end
    
    % Compute the desired output
    control_input = Ubar(1) + delta_U(1, :);
end
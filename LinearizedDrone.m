%% Define the ODE
function X_dot = LinearizedDrone(T, X, A, B)

    % Define the control input at time T
    % if T < 6 && T > 1
    %     U = [5, 0, 5, 0]';
    % else
    %     U = [0, 0, 0, 0]';
    % end
    % U = [-28, 25, -28, 25]';
    % U = [0, 0, 0, 0]';
    U = [0, 0, 0, 0]';

    % Write the state equation
    A = double(A);
    B = double(B);
    X_dot = A*X + B*U;

end 
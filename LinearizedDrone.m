%% Define the ODE
function X_dot = LinearizedDrone(T, X, A, B)

    % Define the control input at time T
    % if T == 1
    %     U = [0, 0, 0, 0]';
    % else
    %     U = [0, 0, 0, 0]';
    % end
    U = [225, 225, 225, 225]';

    % Write the state equation
    A = double(A);
    B = double(B);
    X_dot = A*X + B*U;

end 
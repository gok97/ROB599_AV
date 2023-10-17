%% Define the ODE
function X_dot = LinearizedDrone(T, X, A, B)

    % Define the control input at time T
    U = [0.0008, 0.0008, 0.0008, 0.0008]';

    % Write the state equation
    A = double(A(T));
    B = double(B(T));
    X_dot = A*X + B*U;

end 
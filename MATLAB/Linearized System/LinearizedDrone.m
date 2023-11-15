%% Define the ODE
function X_dot = LinearizedDrone(T, X, A, B)

    % Define the control input at time T
    U = [4.6481e-06, 4.6481e-06, 4.6481e-06, 4.6481e-06]';

    % Write the state equation
    A = double(A);
    B = double(B);
    X_dot = A*X + B*U;

end 
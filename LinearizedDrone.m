%% Define the ODE
function X_dot = LinearizedDrone(T, X)

    % Define simulation constants
    % K = [Ix, Iy, Iz, Ax, Ay, Az, kdx, kdy, kdz, xdot_w, ydot_w, zdot_w, l, kf, km, ka, m, g];
    K = [0.0039875, 0.0078549, 0.0039875, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.25, 3*(10^-5), 0, 0, 0.25, 9.81]'; % BASIC CASE: No air resistance, no gyro effects, no wind

    % Define the equilibrium point
    % XU0 = [x, y, z, u, v, w, phi, theta, psy, p, q, r, w1, w2, w3, w4];
    XU0 = [0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 775, 775, 775, 775]'; % BASIC CASE: Hover

    % Compute the jacobians
    [A, B] = Linearizer(K, XU0, false);

    % Write the state equation
    U = [800, 800, 800, 800]';
    A = double(A);
    B = double(B);

    X_dot = A*X + B*U;

end 
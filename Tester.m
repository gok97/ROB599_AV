%% Prepare workspace
clear
clc

%% Set Variables
% Define simulation constants
% K = [Ix, Iy, Iz, Ax, Ay, Az, kdx, kdy, kdz, xdot_w, ydot_w, zdot_w, l, kf, km, ka, m, g];
K = [0.0039875, 0.0078549, 0.0039875, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.25, 3*(10^-5), 0, 0, 0.25, 9.81]'; % BASIC CASE: No air resistance, no gyro effects, no wind

% Define the equilibrium point
% XU0 = [x, y, z, u, v, w, phi, theta, psy, p, q, r, w1, w2, w3, w4];
XU0 = [0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 775, 775, 775, 775]'; % BASIC CASE: Hover

% Define the initial conditions
t0 = 0;
t1 = 0.1;
X0 = [0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0]; % Drone Hover

%% Compute the nonlinear equations and their linearized counterparts
% Substitute for constants
[tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3] = EquationsOfMotion(K, true);

% Compute the jacobians
[A, B] = Linearizer(tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3, XU0, true);

%% Run simulation
% Solve ODE
% [T_nl,X_nl] = ode45('NonlinearizedDrone',[t0, t1], X0)

% Solve ODE
LinearizedDroneFunction = @(T, X) LinearizedDrone(T, X, A, B);
[T_l,X_l] = ode45(LinearizedDroneFunction,[t0, t1], X0)

%% Plot results

% TEST
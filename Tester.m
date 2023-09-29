%% Prepare workspace
clear
clc

%% Set Variables
% Define simulation constants
% K = [Ix, Iy, Iz, Ax, Ay, Az, kdx, kdy, kdz, xdot_w, ydot_w, zdot_w, l, kf, km, ka, m, g];
K = [7.5*(10^-3), 7.5*(10^-3), 1.3*(10^-2), 0.01, 0.01, 0.045, 0, 0, 0, 0, 0, 0, 0.23, 3.13*(10^-5), 7.5*(10^-7), 0, 0.6461009174, 9.81]'; % BASIC CASE: No air resistance, no gyro effects, no wind

% Define the equilibrium point
% XU0 = [x, y, z, u, v, w, phi, theta, psy, p, q, r, w1, w2, w3, w4];
XU0 = [0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 225, 225, 225, 225]'; % BASIC CASE: Hover
% XU0 = [0, 0, 10, 1, 0, 0, 0, 0, 0, 0, 0, 0, 225, 225, 225, 225]'; % BASIC CASE: Moving Forward 
% XU0 = [0, 0, 10, 1, 0, 1, 0, 0, 0, 0, 0, 0, 225, 225, 225, 225]'; % BASIC CASE: Moving Forward w/ ascent -- CURRENT BEST
% XU0 = [0, 0, 10, 0, 0, 1, 0, 0, 0, 0, 0, 0, 225, 225, 225, 225]'; % BASIC CASE: Pure Ascent
% XU0 = [0, 0, 10, 1, 0, 1, 0, 0, 0, 0, 0, 0.1, 225, 225, 225, 225]'; % BASIC CASE: 
% XU0 = [0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 225, 225, 225, 225]'; % BASIC CASE: Hover

% Define the initial conditionss
t0 = 0;
t1 = 10;
dt = 0.01;
X0 = [0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0]; % Drone Hover
% X0 = [0, 0, 10, 1, 0, 0, 0, 0, 0, 0, 0, 0.1]; % Drone Forward Flight
% X0 = [0, 0, 10, 1, 0, 1, 0, 0, 0, 0, 0, 0]; % Drone Forward Flight w/ Ascent
% X0 = [0, 0, 10, 0, 0, 1, 0, 0, 0, 0, 0, 0]; % Drone Ascent
% X0 = [0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0.1]; % Drone Hover w/ z-spin



%% Compute the nonlinear equations and their linearized counterparts
% Substitute for constants
[tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3] = EquationsOfMotion(K, true);

% Compute the jacobians
[A, B] = Linearizer(tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3, XU0, true)

% Prepare the function handles for the equations of motion
[tk1_function, tk2_function, tk3_function, td1_function, td2_function, td3_function, rk1_function, rk2_function, rk3_function, rd1_function, rd2_function, rd3_function] = Nonlinearizer(tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3);


%% Run simulation
% Solve ODE for nonlinear equations
NonlinearizedDroneFunction = @(Tn, Xn) NonlinearizedDrone(Tn, Xn, tk1_function, tk2_function, tk3_function, td1_function, td2_function, td3_function, rk1_function, rk2_function, rk3_function, rd1_function, rd2_function, rd3_function);
[T_nl,X_nl] = ode45(NonlinearizedDroneFunction, t0: dt: t1, X0)

% Solve ODE for linear equations
LinearizedDroneFunction = @(Tl, Xl) LinearizedDrone(Tl, Xl, A, B);
[T_l,X_l] = ode45(LinearizedDroneFunction,t0: dt: t1, X0)

%% Plot results

% Plot all state variables
summary_fig = figure(1);
labels = ["X (m)", "Y (m)", "Z (m)", "U (m/s)", "V (m/s)", "W (m/s)", "PHI (rad)", "THETA (rad)", "PSI (rad)", "P (rad/s)", "Q (rad/s)", "R (rad/s)"];

for i = 1:(4 * 3)
    % Create a subplot in the ith position
    subplot(4, 3, i);
    
    % Create the line plot for the nonlinear and linear cases
    plot(T_nl, X_nl(:, i));
    hold on;
    plot(T_l, X_l(:, i));
    
    % Add titles or labels as needed
    xlabel('Time (s)');
    ylabel(labels(i));

    if i == 12
        legend(["NL Model", "L Model"])
    end
end
sgtitle('Comparison of NL and L models');

% positions = X_nl(:, 1:3);
% angles= X_nl(:, 7:9);
% l = K(13);
% animate(positions, angles, l)

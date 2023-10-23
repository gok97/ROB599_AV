clear
clc

% Define the state-space system
A = [0.9, 0.1; 0, 0.9];
B = [0.1; 1];
C = [1, 0];
sys = ss(A, B, C, [], 'StateName', {'x1', 'x2'}, 'InputName', 'u', 'OutputName', 'y');

% Define the MPC controller parameters
Ts = 0.1;  % Sampling time
N = 10;    % Prediction horizon
Nu = 3;    % Control horizon

% Create an MPC controller object
mpcObj = mpc(sys, Ts, N, Nu);

% Define constraints (optional)
mpcObj.MV = struct('Min', -1, 'Max', 1);  % Manipulated variable (input) constraints
mpcObj.OV = struct('Min', -Inf, 'Max', Inf);  % Output variable constraints

% Set the weighting matrices for the cost function (Q and R)
Q = diag([1, 1]);  % State weighting
R = 1;             % Control input weighting
% mpcObj.Weights = struct('E', 1, 'CE', 0, 'CU', R, 'MV', 0, 'OV', Q);

% Simulation
T = 20;  % Total simulation time
simulator = sim(mpcObj, T);

% Plot results
t = 0:Ts:T;
figure;
subplot(2, 1, 1);
plot(t, simulator);
xlabel('Time (s)');
ylabel('State Variables');
legend('x1', 'x2');
title('State Variables');

subplot(2, 1, 2);
stairs(t, simulator.u);
xlabel('Time (s)');
ylabel('Control Input (u)');
title('Control Input');

% Note: You may need to adjust the system matrices and tuning parameters to suit your specific system and requirements.

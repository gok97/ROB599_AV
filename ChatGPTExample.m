clear
clc

% Define the system matrices A, B, and C
A = [2, 0.1; 0.3, 0.95];
B = [0.1; 0.2];
C = [1, 0];

% Define the prediction and control horizons
N = 5; % Prediction horizon
Nu = 3; % Control horizon

% Create an LTI object for your system
sys = ss(A, B, C, 0); % 0 represents a direct feedthrough term

% Create an MPC object using the LTI system
mpcobj = mpc(sys, N, Nu);

% Specify constraints on control inputs and state variables (if any)
mpcobj.MV = struct('Min', -1, 'Max', 1);
mpcobj.OV = struct('Min', -1, 'Max', 1);

% Define the reference trajectory
r = 0.5; % Setpoint reference

% Simulation parameters
simTime = 20;
Ts = 1; % Sampling time

% Initialize the state vector
x0 = [0; 0];

% Create arrays to store simulation data
X = zeros(2, simTime);
U = zeros(1, simTime);

% Create an initial MPC state
mpcstate = mpcstate(mpcobj);

for k = 1:simTime
    % Predict the future state using the current state and reference
    [U(k), Info] = mpcmove(mpcobj, mpcstate, r);
    
    % Simulate the system with the control input U(k)
    x_next = A * x0 + B * U(k);
    
    % Store the state for plotting
    X(:, k) = x0;
    
    % Update the current state for the next iteration
    x0 = x_next;
end

% Plot the results
t = 0:Ts:(simTime - 1);
figure;
subplot(2, 1, 1);
plot(t, X(1, :), 'b', 'LineWidth', 2);
ylabel('State x1');
title('MPC Simulation Results');
grid on;

subplot(2, 1, 2);
stairs(t, U, 'r', 'LineWidth', 2);
ylabel('Control Input U');
xlabel('Time Steps');
grid on;

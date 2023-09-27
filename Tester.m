%% Prepare workspace
clear
clc

%% Run simulation

% Set initial conditions
t0 = 0;
t1 = 0.1;
X0 = [0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0]; % Drone Hover + Thrust Application

% Solve ODE
[T,X] = ode45('LinearizedDrone',[t0,t1], X0)

%% Plot results

% TEST
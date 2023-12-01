%% Prepare the Workspace
% Clear the environment
clear
clc

% Add the relevant paths
addpath("NL-MPC\")
addpath("Planning\")

%% Set Simulation Parameters (all in S.I.)
% Environment Settings
env_map = load("city_map.mat").omap3D;      % Binary map defyning the environment
t_const = 0.05;                             % Time increment at which the simulation world is generated
desired_waypoints = [0, 0, 0;
                     1, 1, 1];              % X-Y-Z of waypoints
statespace_limits = [-10   400;             % X-Limits
                     -10   400;             % Y-Limits
                     -10   100;             % Z-Limits
                     inf inf;               % @gokul
                     inf inf;               % @gokul
                     inf inf;               % @gokul
                     inf inf];              % @gokul
wind_mode = "realistic";                    % Wind type: "gust", "breeze", "realistic", "constant"
excess_wind = 3;                            % Excess wind speed
steady_wind = 7;                            % Steady-state wind magnitude
wind_direction = pi()/4;                    % Steady-state wind direction

% Quadcopter settings
v_limit = [-10 10; 
           -10 10; 
           -10 10];                         % Min-Max of the quadcopter velocity in the three directions
a_limit = [-10 10; 
           -10 10; 
           -10 10];                         % Min-Max of the quadcopter acceleration in the three directions
safety_buffer = 1;                          % Buffer by which the map is increased for safe navigation
drop_mode = "delivery";                     % The mode with which the payload will be released: "delivery", "dissemination", "na"
drop_mass = m0*0.5;                         % The mass of the payload to be released

model_constants = {};

% Planner Settings
planner_max_dist = 50;                      % Maximum connecting distance for the planner
planner_goal_bias = 0.8;                    % Bias towards the goal
planner_max_itr = 1000;                     % Maximum number of RRT* Iterations
planner_post_goal = false;                  % Allow the planner to continue planning after having reached a goal
planner_max_nodes = 10000;                  % Maximum tree nodes

planner_constants = {planner_max_dist, planner_goal_bias, planner_max_itr, planner_post_goal, planner_max_nodes};

% Controller Settings
prediction_window = 1;
control_horizon = 1;
control_input_penalty = 1;
control_rate_penalty = 1;
n_states = 12;
n_outputs = 12;
n_inputs = 4;
state_weights = [];
control_limits = [];

controller_constants = {t_const, prediction_window, control_horizon, control_input_penalty, control_rate_penalty, n_states, n_outputs, n_inputs, state_weights, control_limits};

% Options
save = true;                                % Option to save all relevant arrays to .mat files


%% Generate the Desired Trajectory
% Compute the High-Level Trajectory via RRT*
intermediate_waypoints = rrt_star_path_planner(map, desired_waypoints, planner_constants, statespace_limits, safety_buffer);

% Compute the Mid- and Low-Level Trajectory
final_waypoints = trajectory_generator(intermediate_waypoints, t_const, v_limit, a_limit);
total_increments = size(final_waypoints, 1);

% Generate the Corresponding Wind Behaviour
final_wind = wind_generator(wind_mode, excess_wind, wind_step, total_increments, steady_wind, wind_direction);

% Generate the Corresponding Mass Behaviour
final_mass = mass_generator(drop_mode, drop_mass, drop_step, total_increments);

% Save to file if specified
if save == true
    save("reference_trajectory.mat", "final_waypoints")
    save("wind.mat", "final_wind")
    save("mass.mat", "final_mass")
end

%% Execute the Simulation
[actual_state, actual_control, sim_time] = nlmpc_simulator(final_waypoints, referenceControl, final_wind, final_mass, controller_constants, model_constants);

% Save to file if specified
if save == true
    save("actual_trajectory.mat", "actual_state")
    save("actual_inputs.mat", "actual_control")
end

%% Visualize the results



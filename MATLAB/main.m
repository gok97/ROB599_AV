%% Prepare the Workspace
% Clear the environment
clear
clc

% Add the relevant paths
addpath("NL-MPC\")
addpath("Planning\")
addpath("Performance Processors\")

%% Set Simulation Parameters (all in S.I.)
% Environment Settings
env_map = load("city_map.mat").omap3D;      % Binary map defyning the environment
t_const = 0.05;                             % Time increment at which the simulation world is generated
desired_waypoints = [50, 50, 10;
                     250, 250, 50;
                     300, 200, 10];          % X-Y-Z of waypoints
desired_velocities = [];                    % Xdot-Ydot-Zdot Velocities at the designated waypoints
desired_enforcement = [];                   % Specifies whether the velocity between two waypoints should be constant
statespace_limits = [-10   400;             % X-Limits
                     -10   400;             % Y-Limits
                     -10   100;             % Z-Limits
                     inf inf;               % qw quaternion bounds
                     inf inf;               % qx quaternion bounds
                     inf inf;               % qy quaternion bounds
                     inf inf];              % qz quaternion bounds
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

m0 = 0.65;                                  % Dry mass of the quadcopter
Ix0 = 0.0087408;                            % Dry moment of inertia about the x-axis
Iy0 = 0.0087408;                            % Dry moment of inertia about the y-axis
Iz0 = 0.0173188;                            % Dry moment of inertia about the z-axis
Ax = 0.01;                                  % Cross-sectional area through the y-z plane
Ay = 0.01;                                  % Cross-sectional area through the x-z plane
Az = 0.045;                                 % Cross-sectional area through the x-y plane
kdx = 0.1;                                  % Cross-sectional area through the y-z plane
kdy = 0.1;                                  % Rotational drag coefficient about the x-axis
kdz = 0.1;                                  % Rotational drag coefficient about the y-axis
l = 0.23;                                   % Rotational drag coefficient about the z-axis
km = 7.5*(10^-7)/(3.13*(10^-5));            % Moment coefficient for the rotor prop coefficient
ka = 1.0;                                   % Translational drag coefficient
g = 9.81;                                   % Acceleration due to gravity
safety_buffer = 3*l;                        % Buffer by which the map is increased for safe navigation
drop_mode = "delivery";                     % The mode with which the payload will be released: "delivery", "dissemination", "na"
drop_mass = m0*0.5;                         % The mass of the payload to be released
drop_location = [250, 250, 50];             % X-Y-Z of the drop location
kf = 1;

reference_input = ...
    double(sqrt(m0*g/(4*kf)));             % Thrust from each motor at equilibrium (i.e.hover)
control_limits = struct( ...
            Min={0;0;0;0}, ...
            Max={reference_input*5;reference_input*5;reference_input*5;reference_input*5}, ...
            RateMin={-reference_input*5;-reference_input*5;-reference_input*5;-reference_input*5}, ...
            RateMax={reference_input*2.5;reference_input*2.5;reference_input*2.5;reference_input*2.5}...
            );                              % Limits for the control inputs 8and associated rate of change) based on the equilibrium thrust


reference_control = [reference_input, reference_input, reference_input, reference_input];
model_constants = [Ix0, Iy0, Iz0, Ax, Ay, Az, kdx, kdy, kdz, 0, 0, 0, l, kf, km, ka, m0, g];
dry_mass = [m0, Ix0, Iy0, Iz0];

% Planner Settings
planner_max_dist = 50;                      % Maximum connecting distance for the planner
planner_goal_bias = 0.8;                    % Bias towards the goal
planner_max_itr = 1000;                     % Maximum number of RRT* Iterations
planner_post_goal = false;                  % Allow the planner to continue planning after having reached a goal
planner_max_nodes = 10000;                  % Maximum tree nodes

planner_constants = {planner_max_dist, planner_goal_bias, planner_max_itr, planner_post_goal, planner_max_nodes};

% Controller Settings
prediction_window = 15;                      % Prediction window for the MPC
control_horizon = 2;                        % Control window for the MPC
control_input_penalty = 0.1;                  % Penalty weight for each control input
control_rate_penalty = 0;                   % Penalty weight for changing the control input
n_states = 12;                              % Number og states for the system
n_outputs = 12;                             % Number of outputs for the system
n_inputs = 4;                               % Number of control inputs
state_weights = ...
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 ,1];   % Weights for each state (X, Y, Z, Xdot, Ydot, Zdot, phi, theta, psi, p, q, r)             


controller_constants = {t_const, prediction_window, control_horizon, control_input_penalty, control_rate_penalty, n_states, n_outputs, n_inputs, state_weights, control_limits};

% Options
save_bool = true;                                % Option to save all relevant arrays to .mat files
video_mode = "follow";                      % Visualization mode: "follow", "global"
mode = "full";                              % Autonomy mode: "full", "partial"

%% Generate the Desired Trajectory
% Compute the High-Level Trajectory via RRT*
intermediate_waypoints = rrt_star_path_planner(env_map, desired_waypoints, planner_constants, statespace_limits, safety_buffer);

% Compute the Mid- and Low-Level Trajectory
final_waypoints = trajectory_generator(intermediate_waypoints, t_const, v_limit, a_limit);
final_waypoints = final_waypoints';
total_increments = size(final_waypoints, 1);

% Determine the time steps of when the waypoints are reached
drop_counter = 1;
for wp = 1:length(desired_waypoints)
    [~, relevant_index] = min(pdist2(desired_waypoints(wp, :), final_waypoints(:, 1:3)));

    if drop_mode == "delivery"
        if desired_waypoints(wp, :) == drop_location
            drop_step = desired_waypoints(wp, :);
        end
    elseif drop_mode == "dissemination"
        if desired_waypoints(wp, :) == drop_location(1, :) || desired_waypoints(wp, :) == drop_location(2, :)
            drop_step(drop_counter, :) = desired_waypoints(wp, :);
            drop_counter = drop_counter + 1;
        end
    end

    if mode == "partial"
        waypoint_summary(wp, :) = [desired_waypoints(wp, :), desired_velocities(wp, :), desired_enforcement', relevant_index];
    else
        waypoint_summary(wp, :) = [desired_waypoints(wp, :), final_waypoints(4:6, wp)', relevant_index];
    end
end

% Generate the Corresponding Wind Behaviour
final_wind = wind_generator("realistic", excess_wind, 0, total_increments, steady_wind, wind_direction);

% Generate the Corresponding Mass Behaviour
final_mass = mass_generator(drop_mode, drop_mass, drop_step, total_increments, dry_mass);

% Save to file if specified
if save_bool == true
    save("reference_trajectory.mat", "final_waypoints")
    save("wind.mat", "final_wind")
    save("mass.mat", "final_mass")
end

%% Execute the Simulation
[actual_state, actual_control, sim_time] = nlmpc_simulator(final_waypoints, reference_control, final_wind, final_mass, controller_constants, model_constants);

% Save to file if specified
if save_bool == true
    save("actual_trajectory.mat", "actual_state")
    save("actual_inputs.mat", "actual_control")
    disp("Simulation completed in " + num2str(sim_time) + "s.")
end

%% Visualize the results
reporter(t_const, final_waypoints, actual_state, final_wind, final_mass, waypoint_summary, model_constants, video_mode, save_bool, mode);


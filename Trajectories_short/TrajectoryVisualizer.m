%% Prepare workspace
clear
clc

%% Load the data
% Load the waypoints
waypoint_list = load("waypoint_list.mat").waypoint_list;
waypoints = waypoint_list{1};
velocities = waypoint_list{2};
condition = waypoint_list{3};
index_list = waypoint_list{4};

complete_wp = [waypoints, velocities];

% Load the trajectory
xDesired = load("trajectory.mat").xDesired;

% Load the increment indices info
increment_index = load("increment_index.mat").increment_index;

% Load the mass data 
mass_step = load("Trajectories\mass_step.mat").mass_matrix;
mass_ramp = load("Trajectories\mass_ramp.mat").mass_matrix;

% Load the wind data
wind_ramp = load("Trajectories\wind_ramp.mat").wind_matrix;
wind_step = load("Trajectories\wind_step.mat").wind_matrix;
wind_random = load("Trajectories\wind_random.mat").wind_matrix;

trajectory_info = {xDesired, increment_index, mass_step, mass_ramp, wind_ramp, wind_step, wind_random};

%% Visualize
plot_data = {xDesired(1, :), xDesired(2, :), xDesired(3, :), xDesired(4, :), xDesired(5, :), xDesired(6, :), wind_step(:, 1), wind_step(:, 2), mass_step(:, 1), wind_ramp(:, 1), wind_ramp(:, 2), mass_ramp(:, 1)};
tot_inc = 0.05;
T_series = 0:tot_inc:(length(plot_data{1})-1)*tot_inc;
labels = ["X (m)", "Y (m)", "Z (m)", "Xdot (m/s)", "Ydot (m/s)", "Zdot (m/s)", "Wind in X (m/s)", "Wind in Y (m/s)", "Mass (Kg)", "Wind in X (m/s)", "Wind in Y (m/s)", "Mass (Kg)"];        

for i = 1:12
    % Create a subplot in the ith position
    subplot(4, 3, i);

    % Create the line plot for the target and actual cases
    plot(T_series, plot_data{i});

    % Show the tickmarks
    if i < 7
        hold on
        plot(T_series(index_list),complete_wp(:, i), 'rx', 'MarkerSize', 5)
        
    end
    
    % Show when a constant veclotiy is enforced
    if i > 3 && i < 7
        for jdx = 1:length(condition)
            if condition(jdx) == 1
                hold on
                line([T_series(index_list(jdx)), T_series(index_list(jdx+1))], [complete_wp(jdx, i), complete_wp(jdx, i)], 'Color', 'red', 'LineWidth', 2, 'LineStyle', ":");
            end
        end
    end

    % Add titles or labels as needed
    xlabel('Time (s)');
    ylabel(labels(i));
    
    % Adapt limits and tickmarks
    switch i
        case {1, 2}
            ylim([-25, 175]);
            yticks([0, 50, 100, 150]);
        case 3
            ylim([-5, 35]);
            yticks([0, 10, 20, 30]);
        case {4, 5}
            ylim([-5, 35]);
            yticks([0, 10, 20, 30]);
        case{6}
            ylim([-20, 20]);
            yticks([-20, -10, 0, 10, 20]);
        case{7, 8, 10, 11}
            ylim([2.5, 7.5]);
            yticks([2.5,5, 7.5]);
        case {9, 12}
            ylim([0, 2]);
            yticks([0,1, 2]);


    end

end
hold off

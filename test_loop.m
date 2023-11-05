clc;
clear;

trajectory_info = load_trajectory_info(false);

% extract trajectory info
xDesired = trajectory_info{1};
increment_indices = trajectory_info{2};
mass_step = trajectory_info{3};
mass_ramp = trajectory_info{4};
wind_ramp = trajectory_info{5};
wind_step = trajectory_info{6};
wind_random = trajectory_info{7};

% Create list of parameters to loop over
all_mass = {mass_step, mass_ramp};
all_wind = {wind_step, wind_ramp};
pw_list = [5, 10, 15, 20, 25];

for m_idx = 1:1
    for w_idx = 1:1
        for pw_idx = 1:1
            main(xDesired, all_mass{m_idx}, all_wind{w_idx}, pw_list(pw_idx));
        end
    end
end

function trajectory_info = load_trajectory_info(plot_bool)
    % load trajectory info
    xDesired = load("Trajectories\trajectory.mat").xDesired';
    % load increment indices info
    increment_index = load("Trajectories\increment_index.mat").increment_index;
    % load mass info 
    mass_step = load("Trajectories\mass_step.mat").mass_matrix;
    mass_ramp = load("Trajectories\mass_ramp.mat").mass_matrix;
    % load wind info
    wind_ramp = load("Trajectories\wind_ramp.mat").wind_matrix;
    wind_step = load("Trajectories\wind_step.mat").wind_matrix;
    wind_random = load("Trajectories\wind_random.mat").wind_matrix;
    
    trajectory_info = {xDesired, increment_index, mass_step, mass_ramp, wind_ramp, wind_step, wind_random};
    plot_data = {xDesired(:, 1), xDesired(:, 2), xDesired(:, 3), xDesired(:, 4), xDesired(:, 5), xDesired(:, 6), wind_step(:, 1), wind_step(:, 2), mass_step(:, 1), wind_ramp(:, 1), wind_ramp(:, 2), mass_ramp(:, 1)};
    tot_inc = 0.05;
    T_series = 0:tot_inc:(length(plot_data{1})-1)*tot_inc;
    
    if plot_bool
        labels = ["X (m)", "Y (m)", "Z (m)", "Xdot (m/s)", "Ydot (m/s)", "Zdot (m/s)", "X Wind (m/s)", "Y Wind (m/s)", "Mass (Kg)", "X Wind (m/s)", "Y Wind (m/s)", "Mass (Kg)"];        
        for i = 1:12
            % Create a subplot in the ith position
            subplot(4, 3, i);
        
            % Create the line plot for the target and actual cases
            plot(T_series, plot_data{i});
        
            % Add titles or labels as needed
            xlabel('Time (s)');
            ylabel(labels(i));
        end
        hold off
    end
end
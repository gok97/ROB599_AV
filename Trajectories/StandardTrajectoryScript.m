%% Prepare Workspace
clear
clc

%% Settings
% Time interval
dt = 0.05;

% Define the waypoints
waypoints = [0, 0, 0;
            0, 0, 30;
            50, 50, 30;
            100, 100, 30;
            150, 150, 30;
            150, 150, 0.5];

velocities = [0, 0, 0;
            0, 0, 0;
            20, 20, 0;
            20, 20, 0;
            0, 0, 0;
            0, 0, 0];

constant_conditions = [0, 0, 1, 0, 0];

% Define system mass w/o payload
m0 = 0.65;
Ix0 = 0.0087408;
Iy0 = 0.0087408;
Iz0 = 0.0173188;

% Define the mass decrease due to payload release
mdrop = 1.5*m0;

% Define the average wind speed experienced by the quadrotor
wind_0 = 7;

% Define the additional wind speed experienced
wind_extra = 3;
wind_time = 3;

% Define when the step and ramp should be triggered
xyz_drop = [75, 75, 30];
xyz_start = [60, 60, 30];
xyz_end = [90, 90, 30];

%% Generate the necessary trajectories
% Generate the trajectory
trajectory_Vset = QuadrotorRawTrajectoryImproved(dt, waypoints, velocities, constant_conditions);

% Understand the indices when step and ramp need to be triggered
n_samples = length(trajectory_Vset);
[~, n_step] = min(pdist2(xyz_drop, trajectory_Vset(1:3, :)'));
[~, n_start] = min(pdist2(xyz_start, trajectory_Vset(1:3, :)'));
[~, n_end] = min(pdist2(xyz_end, trajectory_Vset(1:3, :)'));
n_delta = n_end - n_start;

% Understand the indices of when the waypoints are reached
for wp = 1:length(waypoints) 
    wp_complete = [waypoints(wp, :), velocities(wp, :)];
    [~, n_wp] = min(pdist2(wp_complete, trajectory_Vset(1:6, :)'));
    index_list(wp) = n_wp;
end

%% Generate the associated mass matrices
for idx = 1:n_samples

    % Generate the mass matrix for a step
    if idx < n_step
        m_step(idx, 1) = m0+mdrop;
    else
        m_step(idx, 1) = m0;
    end
    
    m_step(idx, 2) = Ix0 + 0.00040757*(m_step(idx, 1)-m0);
    m_step(idx, 3) = Iy0 +0.00040757*(m_step(idx, 1)-m0);
    m_step(idx, 4) = Iz0 + 0.00040757*(m_step(idx, 1)-m0);

    % Generate the mass matrix for a ramp
    if idx < n_start
        m_ramp(idx, 1) = m0+mdrop;
    elseif idx < n_end
        m_ramp(idx, 1) = m0+mdrop-(mdrop/n_delta*(idx-n_start));
    else
        m_ramp(idx, 1) = m0;
    end

    m_ramp(idx, 2) = Ix0 + 0.00040757*(m_ramp(idx, 1)-m0);
    m_ramp(idx, 3) = Iy0 +0.00040757*(m_ramp(idx, 1)-m0);
    m_ramp(idx, 4) = Iz0 + 0.00040757*(m_ramp(idx, 1)-m0);

end

%% Generate the associated wind matrices
for idx = 1:n_samples

    % Generate the wind matrix for a step
    if idx < n_step 
        w_step(idx, 1:3) = [wind_0/sqrt(2), wind_0/sqrt(2), 0];
    elseif idx < n_step + 1
        w_step(idx, 1:3) = [(wind_0+wind_extra)/sqrt(2), (wind_0+wind_extra)/sqrt(2), 0];
    else
        w_step(idx, 1:3) = [wind_0/sqrt(2), wind_0/sqrt(2), 0];
    end

    % Generate the wind matrix for a ramp
    if idx < n_start
        w_ramp(idx, 1:3) = [wind_0/sqrt(2), wind_0/sqrt(2), 0];
    elseif idx < n_end
        w_ramp(idx, 1:3) = [(wind_0+wind_extra/n_delta*(idx-n_start))/sqrt(2), (wind_0+wind_extra/n_delta*(idx-n_start))/sqrt(2), 0];
    else
        w_ramp(idx, 1:3) = [(wind_0+wind_extra)/sqrt(2), (wind_0+wind_extra)/sqrt(2), 0];
    end

    % Generate the wind matrix for a the random scenario
    rand_angle = randi([30, 60]);
    w_random(idx, 1:3) = [(wind_0+wind_extra/2*rand)*cosd(rand_angle), (wind_0+wind_extra/2*rand)*sind(rand_angle), 0];

end

%% Save to files
xDesired = trajectory_Vset;
save("trajectory.mat", "xDesired")
mass_matrix = m_step;
save("mass_step.mat", "mass_matrix")
mass_matrix = m_ramp;
save("mass_ramp.mat", "mass_matrix")
wind_matrix = w_step;
save("wind_step.mat", "wind_matrix")
wind_matrix = w_ramp;
save("wind_ramp.mat", "wind_matrix")
wind_matrix = w_random;
save("wind_random.mat", "wind_matrix")

increment_index = [n_step, n_start, n_end];
save("increment_index.mat", "increment_index")

waypoint_list = {waypoints, velocities, constant_conditions, index_list};
save("waypoint_list.mat", "waypoint_list")


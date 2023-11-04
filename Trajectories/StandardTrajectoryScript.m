%% Prepare Workspace
clear
clc

%% Settings
% Time interval
dt = 0.1;

% Define the waypoints
waypoints = [0, 0, 0;
            0, 0, 30;
            10, 10, 30;
            90, 90, 30;
            100, 100, 30;
            100, 100, 0.5];

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
xyz_drop = [50, 50, 30];
xyz_start = [15, 15, 30];
xyz_end = [85, 85, 30];

%% Generate the necessary trajectories
% Generate the trajectory
trajectory_Vset = QuadrotorRawTrajectoryImproved(dt, waypoints, velocities, constant_conditions);

% Understand the indices when step and ramp need to be triggered
n_samples = length(trajectory_Vset);
[~, n_step] = min(pdist2(xyz_drop, trajectory_Vset(1:3, :)'));
[~, n_start] = min(pdist2(xyz_start, trajectory_Vset(1:3, :)'));
[~, n_end] = min(pdist2(xyz_end, trajectory_Vset(1:3, :)'));
n_delta = n_end - n_start;

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
        m_ramp(idx, 1) = m0+mdrop-(mdrop/n_delta*idx);
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
        w_step(idx, 1:3) = [wind_0*sqrt(2), wind_0*sqrt(2), 0];
    elseif idx < n_step + 1
        w_step(idx, 1:3) = [(wind_0+wind_extra)*sqrt(2), (wind_0+wind_extra)*sqrt(2), 0];
    else
        w_step(idx, 1:3) = [wind_0*sqrt(2), wind_0*sqrt(2), 0];
    end

    % Generate the wind matrix for a ramp
    if idx < n_start
        w_ramp(idx, 1:3) = [wind_0*sqrt(2), wind_0*sqrt(2), 0];
    elseif idx < n_end
        w_ramp(idx, 1:3) = [(wind_0+wind_extra/n_delta*(idx-n_start))*sqrt(2), (wind_0+wind_extra/n_delta*(idx-n_start))*sqrt(2), 0];
    else
        w_ramp(idx, 1:3) = [(wind_0+wind_extra)*sqrt(2), (wind_0+wind_extra)*sqrt(2), 0];
    end

    % Generate the wind matrix for a the random scenario
    rand_angle = randi([30, 60]);
    w_random(idx, 1:3) = [(wind_0+wind_extra/2*rand)*cosd(rand_angle), (wind_0+wind_extra/2*rand)*sind(rand_angle), 0];

end

%% Save to files
save("trajectory.mat", "trajectory_Vset")
save("mass_step.mat", "m_step")
save("mass_ramp.mat", "m_ramp")
save("wind_step.mat", "w_step")
save("wind_ramp.mat", "w_ramp")
save("wind_random.mat", "w_random")


%% Prepare workspace
clear
clc

%% Prepare variables
tot_inc = 0.05;
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

%% Execute the trajectory
xdesired = QuadrotorRawTrajectoryImproved(tot_inc, waypoints, velocities, constant_conditions);
T_series = 0:tot_inc:(length(xdesired)-1)*tot_inc;
% Plot all state variables against time
summary_fig = figure(1);
labels = ["X (m)", "Y (m)", "Z (m)", "Xdot (m/s)", "Ydot (m/s)", "Zdot (m/s)", "PHI (rad)", "THETA (rad)", "PSI (rad)", "P (rad/s)", "Q (rad/s)", "R (rad/s)"];


for i = 1:6
    % Create a subplot in the ith position
    subplot(2, 3, i);

    % Create the line plot for the target and actual cases
    plot(T_series, xdesired(i, :));

    % Add titles or labels as needed
    xlabel('Time (s)');
    ylabel(labels(i));

end
hold off

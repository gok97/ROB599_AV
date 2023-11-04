%% Prepare workspace
clear
clc

%% Prepare variables
tot_inc = 0.1;
wp=[0,0,0;
    0,0,1;
    5,0,1
    10,0,1
    15,0,1;
    15,0,0.1]
wv=[0,0,0;
    0,0,0;
    0.5,0,0;
    0.5,0,0;
    0,0,0;
    0,0,0]

wconst = [0; 0; 1; 0; 0]

%% Execute the trajectory
xdesired = QuadrotorRawTrajectoryImproved(tot_inc, wp, wv, wconst);
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

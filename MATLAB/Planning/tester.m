%% Prepare the workspace
clear
clc

%% Define a set of waypoints
wp = [1, 2, 3; 4, 5, 6; 7, 8, 9];

%% Call the trajectory generation function
desiredTrajectory = trajectory_generator(wp, 0.05, [-10 10; -10 10; -10 10], [-10 10; -10 10; -10 10]);

times = 0:0.05:(length(desiredTrajectory)/0.05);

%% Visualize the results
plot3(desiredTrajectory(1, :), desiredTrajectory(2, :), desiredTrajectory(3, :), 'o')
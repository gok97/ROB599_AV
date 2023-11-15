%% Prepare Workspace
clear
clc

%% Settings
tollerance = 0.5;
video_mode = "follow";

%% Load the relevant data
% Load the relevant files
results_path = "C:\Users\feder\OneDrive\Desktop\03 Academics\03 Graduate Education\ROB599\ROB599_AV\Time-Varying Non-Linear MPC\";
current_fnameNL = results_path + "ramp_nl";
current_fnameL = results_path + "ramp_l";
load("increment_index.mat")
load("trajectory.mat")
xHistoryNL = load(current_fnameNL + "_xHistory.mat").xHistory;
xHistoryL = load(current_fnameL + "_xHistory.mat").Xsim;
load(current_fnameNL + "_metadata.mat")

% Ammend first entry being stupid
xHistoryL(1, 3) = 0;

% Load the waypoints
waypoint_list = load("waypoint_list.mat").waypoint_list;
waypoints = waypoint_list{1};
velocities = waypoint_list{2};
condition = waypoint_list{3};
index_list = waypoint_list{4};

complete_wp = [waypoints, velocities];

% Define the time-scale
dt = double(metadata(4));
Ttot = (length(xDesired)-1)*dt;
T_series = 0:dt:Ttot;

%% Compute the error and visualize the 2D error data
% Find the difference between the two arrays for position
deltaNL = sqrt(sum((xDesired(1:3, :)' - xHistoryNL(:, 1:3)).^2, 2));
deltaL = sqrt(sum((xDesired(1:3, :)' - xHistoryL(:, 1:3)).^2, 2));
disp("Displacement Error")
avg_NL = mean(deltaNL)
avg_L = mean(deltaL)

plot(T_series, deltaNL);
hold on
plot(T_series, deltaL);
xlabel('Time (s)');
ylabel("2-Norm between Desired and Actual Position");
legend("NL-MPC", "L-MPC")
title("Displacement Error")
hold off;
pause()

% Find the difference between the two arrays for velocity
deltaNL = sqrt(sum((xDesired(4:6, :)' - xHistoryNL(:, 4:6)).^2, 2));
deltaL = sqrt(sum((xDesired(4:6, :)' - xHistoryL(:, 4:6)).^2, 2));
disp("Velocity Error")
avg_NL = mean(deltaNL)
avg_L = mean(deltaL)

plot(T_series, deltaNL);
hold on
plot(T_series, deltaL);
xlabel('Time (s)');
ylabel("2-Norm between Desired and Actual Position");
legend("NL-MPC", "L-MPC")
title("Velocity Error")
hold off;
pause()

waypoint_performance_summary = [];
for wp_id = 1:length(waypoints)
    waypoint_performance_summary(wp_id, 3) = min(sqrt(sum((xDesired(1:3, index_list(wp_id))' - xHistoryNL(:, 1:3)).^2, 2)));
    waypoint_performance_summary(wp_id, 4) = min(sqrt(sum((xDesired(4:6, index_list(wp_id))' - xHistoryNL(:, 4:6)).^2, 2)));
    waypoint_performance_summary(wp_id, 1) = min(sqrt(sum((xDesired(1:3, index_list(wp_id))' - xHistoryL(:, 1:3)).^2, 2)));
    waypoint_performance_summary(wp_id, 2) = min(sqrt(sum((xDesired(4:6, index_list(wp_id))' - xHistoryL(:, 4:6)).^2, 2)));
end
disp("Waypoint Performance")
disp(waypoint_performance_summary);
pause();

%% Visualize the data in 2D
labels = ["X (m)", "Y (m)", "Z (m)", "Xdot (m/s)", "Ydot (m/s)", "Zdot (m/s)"];        

for i = 1:6
    % Create a subplot in the ith position
    subplot(2, 3, i);

    % Create the line plot for the target and actual cases
    plot(T_series, xDesired(i, :));
    hold on
    plot(T_series, xHistoryNL(:, i));
    plot(T_series, xHistoryL(:, i));

    % Show the tickmarks
    plot(T_series(index_list),complete_wp(:, i), 'rx', 'MarkerSize', 5)


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
    legend("Desired", "NL-MPC", "L-MPC")
    % xlim([14.10, 30])
    % xlim([19.15, 24.25])

    % Adapt limits and tickmarks
    switch i
        case {1, 2}
            ylim([-25, 625]);
            yticks([0, 150, 300, 450, 600]);
        case 3
            ylim([-5, 35]);
            yticks([0, 10, 20, 30]);
        case {4, 5}
            ylim([-5, 35]);
            yticks([0, 10, 20, 30]);
        case{6}
            ylim([-20, 20]);
            yticks([-20, -10, 0, 10, 20]);

    end

end
legend("Desired Trajectory", "Actual Trajectory", 'Position', [0.425, 0.005, 0.2, 0.05])  
legend boxoff  
hold off
pause()

%% Visualize the data in 3D
video_name = "SampleTrajectory.mp4";

% Plot animation
animation_fig = figure(2);
xlabel(labels(1));
ylabel(labels(2));
zlabel(labels(3));
trajectory_line_nl = animatedline('MaximumNumPoints',10000, 'Color','cyan');
trajectory_line_l = animatedline('MaximumNumPoints',10000, 'Color','yellow');

legend("NL Trajectory", "L Trajectory", "Desired Trajectory", "NL Drone", "L Drone", 'Position', [0.1, 0.75, 0.2, 0.05]);


hold on
syms Ix_sym Iy_sym Iz_sym w_x_sym w_y_sym w_z_sym m_sym 
constants = [Ix_sym, Iy_sym, Iz_sym, 0.01, 0.01, 0.045, 0.1, 0.1, 0.1, w_x_sym, w_y_sym, w_z_sym, 0.23, 1, 7.5*(10^-7)/(3.13*(10^-5)), 1.0, m_sym, 9.81]';

drone_shape = [ constants(13)/sqrt(2),  0,              -constants(13)/sqrt(2), constants(13)/sqrt(2),  0,              -constants(13)/sqrt(2);       
                -constants(13)/sqrt(2), 0,              -constants(13)/sqrt(2), constants(13)/sqrt(2),  0,              constants(13)/sqrt(2);
                0,                      0,              0,                      0,                      0,              0;
                1,                      1,              1,                      1,                      1,              1               ];   


writerObj = VideoWriter(video_name, 'Motion JPEG AVI');
writerObj.Quality = 90;
writerObj.FrameRate = 20; % Adjust the frame rate as needed
open(writerObj);

for i = 1:length(T_series)
    % Compute values for the Actual
    % Compute and apply the rotation transformation
    angles_nl = xHistoryNL(i, 7:9);
    R_nl = eul2rotm(angles_nl,'ZYX');

    % Compute the translation
    t_vector_nl = [xHistoryNL(i, 1:3)];

    % Form a homogeneous transformation matrix
    H_nl = [R_nl, t_vector_nl'; 0, 0, 0, 1];
    new_drone_shape_nl = H_nl*drone_shape;


    % Compute and apply the rotation transformation
    angles_l = xHistoryL(i, 7:9);
    R_l = eul2rotm(angles_l,'ZYX');

    % Compute the translation
    t_vector_l = [xHistoryL(i, 1:3)];

    % Form a homogeneous transformation matrix
    H_l = [R_l, t_vector_l'; 0, 0, 0, 1];
    new_drone_shape_l = H_l*drone_shape;

    % Display
    title("Drone Animation - Time: " + num2str(T_series(i)));
    if i == 1
        % Draw a trajectory for the Target
        q = plot3(xDesired(1,:), xDesired(2, :), xDesired(3, :),'k.-');

        p = plot3(new_drone_shape_nl(1,1:6),new_drone_shape_nl(2,1:6),new_drone_shape_nl(3,1:6),'b.-');
        r = plot3(new_drone_shape_l(1,1:6),new_drone_shape_l(2,1:6),new_drone_shape_l(3,1:6),'r.-');

        legend("NL Trajectory", "L Trajectory", "Desired Trajectory", "NL Drone", "L Drone", 'Position', [0.1, 0.75, 0.2, 0.05]);

        % % Generate a tollerance bubble to show if the drone reaches its target
        % for tgt_idx = 1:length(T_series)
        %     if ismember(tgt_idx, index_list)
        %         hold on
        %         % Define the center and radius of the sphere
        %         x_center = xDesired(1, tgt_idx);   % X-coordinate of the center
        %         y_center = xDesired(2, tgt_idx);    % Y-coordinate of the center
        %         z_center = xDesired(3, tgt_idx);    % Z-coordinate of the center
        % 
        % 
        %         radius = tollerance;     % Radius of the sphere
        % 
        %         % Create a grid of points for the sphere
        %         [x, y, z] = sphere(50); % Adjust the resolution as needed
        % 
        %         % Scale the sphere and translate it to the desired location
        %         x = x * radius + x_center;
        %         y = y * radius + y_center;
        %         z = z * radius + z_center;
        % 
        %         % Create a 3D plot of the sphere with semitransparent red color
        %         surf(x, y, z, 'FaceColor', 'k', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
        % 
        % 
        %     end
        % end
        % Set view and lighting options
        lighting gouraud; % Gouraud shading for smooth lighting
        camlight; % Add a light source
    else
        set(p, 'XData', new_drone_shape_nl(1,1:6), 'YData', new_drone_shape_nl(2,1:6),  'ZData', new_drone_shape_nl(3,1:6));
        set(r, 'XData', new_drone_shape_l(1,1:6), 'YData', new_drone_shape_l(2,1:6),  'ZData', new_drone_shape_l(3,1:6));
        legend("NL Trajectory", "L Trajectory", "Desired Trajectory", "NL Drone", "L Drone", 'Position', [0.1, 0.75, 0.2, 0.05]);
    end


    % Update the trajectory line
    addpoints(trajectory_line_nl,xHistoryNL(i,1),xHistoryNL(i,2),xHistoryNL(i,3));
    addpoints(trajectory_line_l,xHistoryL(i,1),xHistoryL(i,2),xHistoryL(i,3));
    hold on

    % Adjust view and set limits
    view(30, 45);
    % legend("Actual", "Target", 'Position', [0.1, 0.05, 0.2, 0.05])
    switch video_mode
        case "follow"
            xlim([min(xHistoryNL(i, 1)-3,xHistoryL(i, 1)-3), max(xHistoryNL(i, 1)+3, xHistoryL(i, 1)+3)])
            ylim([min(xHistoryNL(i, 2)-3,xHistoryL(i, 2)-3), max(xHistoryNL(i, 2)+3, xHistoryL(i, 2)+3)])
            zlim([min(xHistoryNL(i, 3)-3,xHistoryL(i, 3)-3), max(xHistoryNL(i, 3)+3, xHistoryL(i, 3)+3)])
        case "global"
            limiting_min_axis = min(min(min(xHistoryNL(:, 1:3)), min(xDesired(1:3, :))));
            limiting_max_axis = max(max(max(xHistoryNL(:, 1:3)), max(xDesired(1:3, :))));
            deltaNL = limiting_max_axis - limiting_min_axis;
            avg_x = (mean([xHistoryNL(:, 1), xDesired(1, :)]));
            avg_y = (mean([xHistoryNL(:, 2), xDesired(2, :)]));
            avg_z = (mean([xHistoryNL(:, 3), xDesired(3, :)]));
            xlim([min(avg_x) - deltaNL/2 - 10, max(avg_x) + deltaNL/2 + 10])
            ylim([min(avg_y) - deltaNL/2 - 10, max(avg_y) + deltaNL/2 + 10])
            zlim([min(avg_z) - deltaNL/2 - 10, max(avg_z) + deltaNL/2 + 10])
    end
    

    % Capture the current frame
    frame = getframe(animation_fig);

    % Write the frame to the video
    writeVideo(writerObj, frame);

    % Fix framerate to match rate of sampling and update plot
    pause(dt);
    drawnow;
end


% Save the video
close(writerObj);





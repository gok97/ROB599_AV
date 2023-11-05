%% Prepare Workspace
clear
clc

%% Settings
tollerance = 1;
video_mode = "follow";
%% Load the relevant data
% Load the relevant files
current_fname = "";
load("increment_index.mat")
load("trajectory.mat")
load(current_fname + "_xHistory.mat")
load(current_fname + "_metadata.mat")

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
% Find the difference between the two arrays
delta = sqrt(sum((xDesired(1:6, :)' - xHistory(:, 1:6)).^2, 2));
plot(T_series, delta);
xlabel('Time (s)');
ylabel("2-Norm between Desired and Actual Position");
pause()

%% Visualize the data in 2D
labels = ["X (m)", "Y (m)", "Z (m)", "Xdot (m/s)", "Ydot (m/s)", "Zdot (m/s)"];        

for i = 1:6
    % Create a subplot in the ith position
    subplot(2, 3, i);

    % Create the line plot for the target and actual cases
    plot(T_series, xDesired(i, :));
    hold on
    plot(T_series, xHistory(:, i));

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

    end

end
legend("Desired Trajectory", "Actual Trajectory", 'Position', [0.425, 0.005, 0.2, 0.05])  
legend boxoff  
hold off
pause()

%% Visualize the data in 3D

video_name = "SampleTrajectory.avi";

% Plot animation
animation_fig = figure(2);
xlabel(labels(1));
ylabel(labels(2));
zlabel(labels(3));
trajectory_line_nl = animatedline('MaximumNumPoints',10000, 'Color','cyan');
hold on
syms Ix_sym Iy_sym Iz_sym w_x_sym w_y_sym w_z_sym m_sym 
constants = [Ix_sym, Iy_sym, Iz_sym, 0.01, 0.01, 0.045, 0.1, 0.1, 0.1, w_x_sym, w_y_sym, w_z_sym, 0.23, 1, 7.5*(10^-7)/(3.13*(10^-5)), 1.0, m_sym, 9.81]';
            
drone_shape = [ constants(13)/sqrt(2),  0,              -constants(13)/sqrt(2), constants(13)/sqrt(2),  0,              -constants(13)/sqrt(2);       
                -constants(13)/sqrt(2), 0,              -constants(13)/sqrt(2), constants(13)/sqrt(2),  0,              constants(13)/sqrt(2);
                0,                      0,              0,                      0,                      0,              0;
                1,                      1,              1,                      1,                      1,              1               ];   


writerObj = VideoWriter(video_name, 'Motion JPEG AVI');
writerObj.Quality = 90;
writerObj.FrameRate = 100; % Adjust the frame rate as needed
open(writerObj);

for i = 1:length(T_series)
    % Compute values for the Actual
    % Compute and apply the rotation transformation
    angles_nl = xHistory(i, 7:9);
    R_nl = eul2rotm(angles_nl,'ZYX');

    % Compute the translation
    t_vector_nl = [xHistory(i, 1:3)];

    % Form a homogeneous transformation matrix
    H_nl = [R_nl, t_vector_nl'; 0, 0, 0, 1];
    new_drone_shape_nl = H_nl*drone_shape;

    % Display
    title("Drone Animation - Time: " + num2str(T_series(i)));
    if i == 1
        p = plot3(new_drone_shape_nl(1,1:6),new_drone_shape_nl(2,1:6),new_drone_shape_nl(3,1:6),'b.-');

        % Draw a trajectory for the Target
        q = plot3(xDesired(1,:), xDesired(2, :), xDesired(3, :),'r.-');
        
        % Generate a tollerance bubble to show if the drone reaches its target
        for tgt_idx = 1:length(T_series)
            if ismember(tgt_idx, index_list)
                hold on
                % Define the center and radius of the sphere
                x_center = xDesired(1, tgt_idx);   % X-coordinate of the center
                y_center = xDesired(2, tgt_idx);    % Y-coordinate of the center
                z_center = xDesired(3, tgt_idx);    % Z-coordinate of the center
                radius = tollerance;     % Radius of the sphere
                
                % Create a grid of points for the sphere
                [x, y, z] = sphere(50); % Adjust the resolution as needed
                
                % Scale the sphere and translate it to the desired location
                x = x * radius + x_center;
                y = y * radius + y_center;
                z = z * radius + z_center;
                
                % Create a 3D plot of the sphere with semitransparent red color
                surf(x, y, z, 'FaceColor', 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
                

            end
        end
        % Set view and lighting options
        lighting gouraud; % Gouraud shading for smooth lighting
        camlight; % Add a light source
    else
        set(p, 'XData', new_drone_shape_nl(1,1:6), 'YData', new_drone_shape_nl(2,1:6),  'ZData', new_drone_shape_nl(3,1:6));
    end



    % addpoints(trajectory_line_nl,xHistory(i,1),xHistory(i,2),xHistory(i,3));

    % Adjust view and set limits
    view(30, 45);
    % legend("Actual", "Target", 'Position', [0.1, 0.05, 0.2, 0.05])
    switch video_mode
        case "follow"
            xlim([min(xHistory(i, 1)-3,xDesired(1, i)-3), max(xHistory(i, 1)+3, xDesired(1, i)+3)])
            ylim([min(xHistory(i, 2)-3,xDesired(2, i)-3), max(xHistory(i, 2)+3, xDesired(2, i)+3)])
            zlim([min(xHistory(i, 3)-3,xDesired(3, i)-3), max(xHistory(i, 3)+3, xDesired(3, i)+3)])
        case "global"
            limiting_min_axis = min(min(min(xHistory(:, 1:3)), min(xDesired(1:3, :))));
            limiting_max_axis = max(max(max(xHistory(:, 1:3)), max(xDesired(1:3, :))));
            delta = limiting_max_axis - limiting_min_axis;
            avg_x = (mean([xHistory(:, 1), xDesired(1, :)]));
            avg_y = (mean([xHistory(:, 2), xDesired(2, :)]));
            avg_z = (mean([xHistory(:, 3), xDesired(3, :)]));
            xlim([min(avg_x) - delta/2 - 10, max(avg_x) + delta/2 + 10])
            ylim([min(avg_y) - delta/2 - 10, max(avg_y) + delta/2 + 10])
            zlim([min(avg_z) - delta/2 - 10, max(avg_z) + delta/2 + 10])
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



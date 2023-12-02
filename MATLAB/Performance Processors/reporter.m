% Function to report key statistics and visualize the data 
function reporter(timeConstant, desiredTrajectory, actualTrajectory, windMatrix, massMatrix, waypointSummary, modelConstants, videoMode, save_bool, mode)

% Define the time-scale
tTot = (length(desiredTrajectory)-1)*timeConstant;
tSeries = 0:timeConstant:tTot;

% Determine the difference between the desired and executed trajectories
deltaPosition = sqrt(sum((desiredTrajectory(:, 1:3) - actualTrajectory(:, 1:3)).^2, 2));
deltaVelocity = sqrt(sum((desiredTrajectory(:, 4:6) - actualTrajectory(:, 4:6)).^2, 2));
disp("Average Displacement Error: " + num2str(mean(deltaPosition)) + "m.")
disp("Average Velocity Error: " + num2str(mean(deltaVelocity)) + "m/s.")


% Determine how close to each waypoint the quadcopter gets
wpPerformSummary = [];
for wp_id = 1:size(waypointSummary, 1)
    wpPerformSummary(wp_id, 1) = min(sqrt(sum((desiredTrajectory(waypointSummary(wp_id, end), 1:3) - actualTrajectory(:, 1:3)).^2, 2)));
    wpPerformSummary(wp_id, 2) = min(sqrt(sum((desiredTrajectory(waypointSummary(wp_id, end), 4:6) - actualTrajectory(:, 4:6)).^2, 2)));
end
if mode == "partial"
    disp("Displacement (m) and Velocity (m/s) Error for Each Waypoint:")
    disp(wpPerformSummary);
else
    disp("Displacement (m) Error for Each Waypoint:")
    disp(wpPerformSummary(:, 1));
end

% Determine if the quadcopter hits any obstacles
%@gokul
pause();

% Set labels for plotting
labels = ["X (m)", "Y (m)", "Z (m)", "Xdot (m/s)", "Ydot (m/s)", "Zdot (m/s)", "Xdot Wind (m/s)", "Ydot Wind (m/s)", "Mass (kg)"];    

% Plot the difference between the desired and executed trajectories across time
deltaFig = figure(1);
subplot(1, 2, 1);
plot(tSeries, deltaPosition);
hold on
xlabel('Time (s)');
ylabel("2-Norm between Reference and Actual Position");
title("Displacement Error")
subplot(1, 2, 2);
plot(tSeries, deltaVelocity);
hold on
xlabel('Time (s)');
ylabel("2-Norm between Reference and Actual Velocity");
title("Velocity Error")
if save_bool == true
    saveas(deltaFig, "delta.jpg", 'jpeg');
end
pause();

% Plot the desired and executed relevant states across time, alongside the wind and mass dsturbances
stateFig = figure(2);

for i = 1:6
    % Create a subplot in the ith position
    subplot(3, 3, i);

    % Create the line plot for the target and actual cases
    plot(tSeries, desiredTrajectory(:, i));
    hold on
    plot(tSeries, actualTrajectory(:, i));

    % Show the tickmarks
    if mode == "partial"
        plot(tSeries(waypointSummary(:, end)),waypointSummary(:, i), 'rx', 'MarkerSize', 5)
        if i>3
            for jdx = 1:length(waypointSummary(:, end-1))-1
                    if waypointSummary(jdx, end-1) == 1
                        hold on
                        line([tSeries(waypointSummary(jdx, end)), tSeries(waypointSummary(jdx+1, end))], [waypointSummary(jdx, i), waypointSummary(jdx, i)], 'Color', 'red', 'LineWidth', 2, 'LineStyle', ":");
                    end
            end
        end
    else
        if i<4
            plot(tSeries(waypointSummary(:, end)),waypointSummary(:, i), 'rx', 'MarkerSize', 5)
        end

    end

    % Add titles or labels as needed
    xlabel('Time (s)');
    ylabel(labels(i));
    legend("Reference", "Actual")

end

% Create a subplot for the wind in the X
subplot(3, 3, 7);
plot(tSeries, windMatrix(:, 1));
hold on
xlabel('Time (s)');
ylabel(labels(7));

% Create a subplot for the wind in the Y
subplot(3, 3, 8);
plot(tSeries, windMatrix(:, 2));
hold on
xlabel('Time (s)');
ylabel(labels(8));

% Create a subplot for the mass
subplot(3, 3, 9);
plot(tSeries, massMatrix(:, 1));
hold on
xlabel('Time (s)');
ylabel(labels(9));

if save_bool == true
    saveas(stateFig, "state.jpg", 'jpeg');
end
pause();

% Simulate the drone navigating the commanded trajectory
videoFig = figure(3);
xlabel(labels(1));
ylabel(labels(2));
zlabel(labels(3));
trajectoryLine = animatedline('MaximumNumPoints',10000, 'Color','cyan');
hold on
legend("Actual", "Reference", "Quadcopter", 'Position', [0.1, 0.75, 0.2, 0.05]);
quadcopterShape = [ modelConstants(13)/sqrt(2),     0,              -modelConstants(13)/sqrt(2),    modelConstants(13)/sqrt(2),     0,              -modelConstants(13)/sqrt(2);       
                    -modelConstants(13)/sqrt(2),    0,              -modelConstants(13)/sqrt(2),    modelConstants(13)/sqrt(2),     0,              modelConstants(13)/sqrt(2);
                    0,                              0,              0,                              0,                              0,              0;
                    1,                              1,              1,                              1,                              1,              1                           ];  

if save_bool == true
    writerObj = VideoWriter("simulation.mp4", 'Motion JPEG AVI');
    writerObj.Quality = 90;
    writerObj.FrameRate = 20; % Adjust the frame rate as needed
    open(writerObj);
end

for i = 1:length(tSeries)
    % Compute and apply the rotation transformation
    angles = actualTrajectory(i, 7:9);
    RMatrix = eul2rotm(angles,'ZYX');

    % Compute the translation
    tVector = [actualTrajectory(i, 1:3)];

    % Form a homogeneous transformation matrix
    HMatrix = [RMatrix, tVector'; 0, 0, 0, 1];
    newQuadcopterShape = HMatrix*quadcopterShape;

    % Display
    title("Drone Animation - Time: " + num2str(tSeries(i)));
    if i == 1
        % Draw a trajectory for the Target
        q = plot3(desiredTrajectory(:, 1), desiredTrajectory(:, 2), desiredTrajectory(:, 3),'k.-');
        p = plot3(newQuadcopterShape(1,1:6),newQuadcopterShape(2,1:6),newQuadcopterShape(3,1:6),'b.-');

        legend("Actual", "Reference", "Quadcopter", 'Position', [0.1, 0.75, 0.2, 0.05]);

        % Set view and lighting options
        lighting gouraud;
        camlight;

    else
        set(p, 'XData', newQuadcopterShape(1,1:6), 'YData', newQuadcopterShape(2,1:6),  'ZData', newQuadcopterShape(3,1:6));
        legend("Actual", "Reference", "Quadcopter", 'Position', [0.1, 0.75, 0.2, 0.05]);
    end

    % Update the trajectory line
    addpoints(trajectoryLine, actualTrajectory(i,1), actualTrajectory(i,2), actualTrajectory(i,3));
    hold on

    % Adjust view and set limits
    view(30, 45);
    switch videoMode
        case "follow"
            xlim([actualTrajectory(i, 1)-3, actualTrajectory(i, 1)+3])
            ylim([actualTrajectory(i, 2)-3, actualTrajectory(i, 2)+3])
            zlim([actualTrajectory(i, 3)-3, actualTrajectory(i, 3)+3])
        case "global"
            limitingMinAxis = min(min(actualTrajectory(:, 1:3)));
            limitingMaxAxis = max(max(actualTrajectory(:, 1:3)));
            deltaAxis = limitingMaxAxis - limitingMinAxis;
            avgX = (mean([actualTrajectory(:, 1), desiredTrajectory(1, :)]));
            avgY = (mean([actualTrajectory(:, 2), desiredTrajectory(2, :)]));
            avgZ = (mean([actualTrajectory(:, 3), desiredTrajectory(3, :)]));
            xlim([min(avgX) - deltaAxis/2 - 10, max(avgX) + deltaAxis/2 + 10])
            ylim([min(avgY) - deltaAxis/2 - 10, max(avgY) + deltaAxis/2 + 10])
            zlim([min(avgZ) - deltaAxis/2 - 10, max(avgZ) + deltaAxis/2 + 10])
    end
    
    if save_bool == true
        frame = getframe(videoFig);
        writeVideo(writerObj, frame);
    end

    % Fix framerate to match rate of sampling and update plot
    pause(timeConstant);
    drawnow;
end

if save_bool == true
    close(writerObj);
end

end




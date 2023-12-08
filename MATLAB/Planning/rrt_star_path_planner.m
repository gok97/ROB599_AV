% Function to generate a high-level trajectory using RRT* bteween specified waypoints in a provided map
function [plannerWaypoints] = rrt_star_path_planner(map, waypointsList, plannerConstants, stateSpaceLimits, safetyBuffer)
    
    % Set seed for repeatability
    rng(100, "twister");

    % Augment waypoints with orientation (0 degrees of roll, pitch and yaw)
    waypointsList = [waypointsList repmat([1 0 0 0], size(waypointsList, 1), 1)];    
    
    % Define the state space object 
    ss = stateSpaceSE3(stateSpaceLimits);

    % Set the map thresholds and inflate the borders by the quadcopter's safety_buffer
    map.FreeThreshold = map.OccupiedThreshold;
    inflate(map, safetyBuffer);

    % Define a state validator object
    sv = validatorOccupancyMap3D(ss,Map=map);
    sv.ValidationDistance = 0.1;

    % Verify is the input waypoints are valid (not in collision with
    % obstacles)
    waypointsValidity = isStateValid(sv,waypointsList);
    valid = all(waypointsValidity);
    if valid == 0
        invalid_waypoints_error_msg = "The input waypoints are invalid!";
        disp("The following indices of input waypoints are in collision:")
        invalid_waypoints_idx = find(waypointsValidity == 0);
        disp(invalid_waypoints_idx)
        error(invalid_waypoints_error_msg)
    end

    % Set up the RRT* Path Planner
    planner = plannerRRTStar(ss,sv);
    planner.MaxConnectionDistance = plannerConstants{1};
    planner.GoalBias = plannerConstants{2};
    planner.MaxIterations = plannerConstants{3};
    planner.ContinueAfterGoalReached = plannerConstants{4};
    planner.MaxNumTreeNodes = plannerConstants{5};

    % Plan for the given waypoints
    plannerWaypoints = [];
    for i=1:size(waypointsList, 1)-1
        % Set the start and end poses
        startPose = waypointsList(i, :);
        goalPose = waypointsList(i+1, :);
    
        % Execute the path planning
        [pathObject, solutionInfo] = plan(planner,startPose,goalPose);
        plannerWaypoints = [plannerWaypoints; pathObject.States(1:end-1, :)];

        % Check if a path is found
        if (~solutionInfo.IsPathFound)
            disp("No Path Found by the RRT*, terminating...")
            break
        end

    end

    % Append the final waypoint
    plannerWaypoints = [plannerWaypoints; waypointsList(end, :)];

end
function rrt_star_path_planner(omap3D, waypoints, safety_buffer)
    omap3D.FreeThreshold = omap3D.OccupiedThreshold;
    % Define state space object 
    ss = stateSpaceSE3([-10   400;
                        -10   400;
                        -10   100;
                        inf inf;
                        inf inf;
                        inf inf;
                        inf inf]);
    
    % Add buffer region around the obstacles
    % inflate(omap3D, safety_buffer);

    % Define State Validator Object
    sv = validatorOccupancyMap3D(ss,Map=omap3D);
    sv.ValidationDistance = 0.1;

    % Set up RRT* Path Planner
    planner = plannerRRTStar(ss,sv);
    planner.MaxConnectionDistance = 50;
    planner.GoalBias = 0.8;
    planner.MaxIterations = 1000;
    planner.ContinueAfterGoalReached = true;
    planner.MaxNumTreeNodes = 10000;

    %
    startPose = [1 1 5 0.7 1 0 0];
    % goalPose = [350 350 80 1 0 0 0];
    
    % startPose = [12 22 25 0.7 0.2 0 0.1];  % [x y z qw qx qy qz]
    goalPose = [150 180 35 0.3 0 0.1 0.6];

    % Execute path planning
    [pthObj, solnInfo] = plan(planner,startPose,goalPose);
    
    % Check if a path is found
    if (~solnInfo.IsPathFound)
        disp("No Path Found by the RRT*, terminating...")
        return
    end

    % Plot map, start pose, and goal pose
    show(omap3D)
    hold on
    scatter3(startPose(1),startPose(2),startPose(3),100,".r")
    scatter3(goalPose(1),goalPose(2),goalPose(3),100,".g")
    
    % Plot path computed by path planner
    plot3(pthObj.States(:,1),pthObj.States(:,2),pthObj.States(:,3),"-g")
    view([-31 63])
    legend("","Start Position","Goal Position","Planned Path")
    hold off
end
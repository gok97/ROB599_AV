% Function to generate a time-optimal, minimum-snap trajectory
function desiredTrajectory = trajectory_generator(RRT_waypoints, time_constant, vel_limits, acc_limits)
    %% Compute a mid-level trajectory with a minimum-time planner
    [intermediate_pt,intermediate_vel,intermediate_acc,intermediate_t] = contopptraj(RRT_waypoints, vel_limits, acc_limits);

    %% Determine the RRT waypoints's time and required number of samples
    for idx = 1:size(RRT_waypoints, 1)
        [~ , relevant_index] = min(pdist2(intermediate_pt', RRT_waypoints(idx,:)));
        time_waypoint(idx) = intermediate_t(relevant_index);
    end

    n_samples = ceil(time_waypoint(end)/time_constant);

    %% Compute the low-level trajectory with a minimum-snap planner
    [desired_pt,desired_vel,qdd,qddd,qdddd,pp,tPoints,tSamples] = minsnappolytraj(RRT_waypoints, time_waypoint, n_samples);
    

    %% Combine to form the desiderd trajectory
    desiredTrajectory = zeros(12, n_samples);
    desiredTrajectory(1:3, :) = desired_pt;
    desiredTrajectory(4:6, :) = desired_vel;
end
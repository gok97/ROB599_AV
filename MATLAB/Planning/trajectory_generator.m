% Function to generate a time-optimal, minimum-snap trajectory
function desiredTrajectory = trajectory_generator(RRTWaypoints, timeConstant, velLimits, accLimits)
    % Compute a mid-level trajectory with a minimum-time planner
    [intermediatePt,intermediateVel,intermediateAcc,intermediateT] = contopptraj(RRTWaypoints, velLimits, accLimits);

    % Determine the RRT waypoints's time and required number of samples
    for idx = 1:size(RRTWaypoints, 1)
        [~ , relevantIndex] = min(pdist2(intermediatePt', RRTWaypoints(idx,:)));
        timeWaypoint(idx) = intermediateT(relevantIndex);
    end

    % Determine the required number of samples
    nSamples = ceil(timeWaypoint(end)/timeConstant);

    % Compute the low-level trajectory with a minimum-snap planner
    [desiredPt,desiredVel,qdd,qddd,qdddd,pp,tPoints,tSamples] = minsnappolytraj(RRTWaypoints, timeWaypoint, nSamples);
    
    % Combine to form the desiderd trajectory
    desiredTrajectory = zeros(12, nSamples);
    desiredTrajectory(1:3, :) = desiredPt;
    desiredTrajectory(4:6, :) = desiredVel;
end
function [xdesired] = QuadrotorRawTrajectory(increments, waypoints, wayvelocities)
% This function generates the raw trajectory based on a target njmber of increments, 
% a set of waypoints, and if specified, a set of wayvelocities.


% Determine the target trajectory based on the inputs. Assume the waypoints
% are uniformly distributed. The wayvelocities will enforce the speed and
% consequently the time profile.
inc_per_seg = (increments/(length(waypoints)-1))-1;
default_value = 5;

% Set the target points
tot_counter = 1;
for idx = 2:length(waypoints)

    % Compute the distance to move through
    dx = (waypoints(idx, 1) - waypoints(idx-1, 1))/inc_per_seg;
    dy = (waypoints(idx, 2) - waypoints(idx-1, 2))/inc_per_seg;
    dz = (waypoints(idx, 3) - waypoints(idx-1, 3))/inc_per_seg;
    
    % Add intermediate points
    for jdx = 0:inc_per_seg
        x(tot_counter) = waypoints(idx-1, 1) + dx*jdx;
        y(tot_counter) = waypoints(idx-1, 2) + dy*jdx;
        z(tot_counter) = waypoints(idx-1, 3) + dz*jdx;
        tot_counter = tot_counter + 1;
    end
end

% Determine the target velocities based on the inputs
tot_counter = 1;
for idx = 2:length(wayvelocities) 

    % Compute the deltas
    dx = (waypoints(idx, 1) - waypoints(idx-1, 1))/inc_per_seg;
    dy = (waypoints(idx, 2) - waypoints(idx-1, 2))/inc_per_seg;
    dz = (waypoints(idx, 3) - waypoints(idx-1, 3))/inc_per_seg;

    dxdot = (wayvelocities(idx, 1) - wayvelocities(idx-1, 1))/inc_per_seg;
    dydot = (wayvelocities(idx, 2) - wayvelocities(idx-1, 2))/inc_per_seg;
    dzdot = (wayvelocities(idx, 3) - wayvelocities(idx-1, 3))/inc_per_seg;

    % Add intermediate points
    for jdx = 0:inc_per_seg
        if jdx > (inc_per_seg/2)
            jdx_ref = (2 - jdx/(inc_per_seg/2));
        else
            jdx_ref = jdx/(inc_per_seg/2);
        end

        if dxdot == 0 && wayvelocities(idx-1, 1) == 0 && dx ~=0
            xdot(tot_counter) = jdx_ref * default_value;
        else
            xdot(tot_counter) = wayvelocities(idx-1, 1) + dxdot*jdx;
        end

        if dydot == 0 && wayvelocities(idx-1, 2) == 0 && dy ~=0
            ydot(tot_counter) = jdx_ref * default_value;
        else
            ydot(tot_counter) = wayvelocities(idx-1, 2) + dydot*jdx;
        end

        if dzdot == 0 && wayvelocities(idx-1, 3) == 0 && dz ~=0
            zdot(tot_counter) = jdx_ref * default_value;
        else
            zdot(tot_counter) = wayvelocities(idx-1, 3) + dzdot*jdx;

        end

        tot_counter = tot_counter + 1;
    end
end
        


% Add any missing rows
if rem(inc_per_seg,1) ~= 0
    current_len = length(x);

    while length(x) ~= increments
        x(current_len + 1) = waypoints(end, 1);
        y(current_len + 1) = waypoints(end, 2);
        z(current_len + 1) = waypoints(end, 3);
        xdot(current_len + 1) = wayvelocities(end, 1);
        ydot(current_len + 1) = wayvelocities(end, 2);
        zdot(current_len + 1) = wayvelocities(end, 3);

        current_len = current_len + 1;
    end
end

% Leave the drone pose as unspecified
phi = zeros(1,increments);
theta = zeros(1,increments);
psi = zeros(1,increments);
phidot = zeros(1,increments);
thetadot = zeros(1,increments);
psidot = zeros(1,increments);

% Populate the vector
xdesired = [x;y;z;xdot;ydot;zdot;phi;theta;psi;phidot;thetadot;psidot];

end


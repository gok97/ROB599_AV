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
if isempty(wayvelocities)
    % If not specified, determine a target velocity trivially.
    xdot = zeros(1,increments);
    ydot = zeros(1,increments);
    zdot = zeros(1,increments);
    
    for i = 1:(increments-1)
        xdot(i) = (x(i+1)-x(i))/((i+1)-(i));
        ydot(i) = (y(i+1)-y(i))/((i+1)-(i));
        zdot(i) = (z(i+1)-z(i))/((i+1)-(i));
    end
    
    xdot(increments) = x(increments-1);
    ydot(increments) = y(increments-1);
    zdot(increments) = z(increments-1);

else
    % If specified, enforce a simple constant acceleration model
    tot_counter = 1;
    for idx = 2:length(wayvelocities)        
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

            if dxdot == 0 && wayvelocities(idx-1, 1) == 0
                xdot(tot_counter) = jdx_ref * default_value;
            else
                xdot(tot_counter) = wayvelocities(idx-1, 1) + dxdot*jdx;
            end

            if dydot == 0 && wayvelocities(idx-1, 2) == 0
                ydot(tot_counter) = jdx_ref * default_value;
            else
                ydot(tot_counter) = wayvelocities(idx-1, 2) + dydot*jdx;
            end

            if dzdot == 0 && wayvelocities(idx-1, 3) == 0
                zdot(tot_counter) = jdx_ref * default_value;
            else
                zdot(tot_counter) = wayvelocities(idx-1, 3) + dzdot*jdx;

            end

            tot_counter = tot_counter + 1;
        end
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


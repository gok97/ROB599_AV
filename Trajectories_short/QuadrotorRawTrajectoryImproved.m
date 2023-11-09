function [xdesired] = QuadrotorRawTrajectoryImproved(delta_increment, waypoints, wayvelocities, constant_settings)
% This function generates the raw trajectory based on a target njmber of increments, 
% a set of waypoints, and if specified, a set of wayvelocities.

% Determine the resolution of the delta increment
str_x = num2str(delta_increment);
decimal_position = strfind(str_x, '.');
if isempty(decimal_position)
    decimal_places = 0;
else
    decimal_places = length(str_x) - decimal_position;
end

% Assume a constant max acceleration motion model
absolute_max_acceleration = 10;

% For each segment, determine how much time is needed
xdesired = [];
for idx = 2:length(waypoints)
    % Compute the delta displacement and velocity
    dx(1) = (waypoints(idx, 1) - waypoints(idx-1, 1));
    dx(2) = (waypoints(idx, 2) - waypoints(idx-1, 2));
    dx(3) = (waypoints(idx, 3) - waypoints(idx-1, 3));
    dxdot(1) = (wayvelocities(idx, 1) - wayvelocities(idx-1, 1));
    dxdot(2) = (wayvelocities(idx, 2) - wayvelocities(idx-1, 2));
    dxdot(3) = (wayvelocities(idx, 3) - wayvelocities(idx-1, 3));

    % Solve a simple system of equations for each direction to detremine the limiting time
    dta_max = 0;
    dtb_max = 0;

    if not(constant_settings(idx-1))
        for eqt = 1:3
            % Determine the nature of the acceleration
            max_acceleration = absolute_max_acceleration*sign(dx(eqt));
    
            syms dta dtb
            eq1 = dxdot(eqt) == max_acceleration*dta - max_acceleration*dtb;
            eq2 = dx(eqt) == wayvelocities(idx-1, eqt)*(dta+dtb) + 0.5*max_acceleration*dta^2 + (max_acceleration*dta)*dtb - 0.5*max_acceleration*dtb^2;
            [dta_val, dtb_val] = solve([eq1, eq2], [dta, dtb]);
    
             % Choose the positive solution
            if double(dta_val(1)) >= 0
                dta_val = double(dta_val(1));
            else
                dta_val = dta_val(2);
            end
    
            if double(dtb_val(1)) >= 0
                dtb_val = double(dtb_val(1));
            else
                dtb_val = dtb_val(2);
            end
    
            if dta_val > dta_max
                dta_max = dta_val;
            end
    
            if dtb_val > dtb_max
                dtb_max = dtb_val;
            end
        end
    
        % Determine the neccessary number of increments
        n_increments_a = (round(dta_max * 10^decimal_places) / 10^decimal_places)/delta_increment;
        n_increments_b = (round(dtb_max * 10^decimal_places) / 10^decimal_places)/delta_increment;
        
        % Place the mid-waypoint targets
        temp_xdesired = [];
    
        for eqt = 1:3
            v_prev = wayvelocities(idx-1, eqt);
            x_prev = waypoints(idx-1, eqt);
    
            for jdx = 1:n_increments_a+n_increments_b
                % Find the adjusted acceleration
                syms curr_acc
                eq3 = dx(eqt) == wayvelocities(idx-1, eqt)*(dta_max+dtb_max) + 0.5*curr_acc*dta_max^2 + (curr_acc*dta_max)*dtb_max - 0.5*curr_acc*dtb_max^2;
                current_acceleration = double(solve(eq3, curr_acc));
    
                if jdx <= n_increments_a
                    current_acceleration = current_acceleration;
                else
                    current_acceleration = -current_acceleration;
                end

                temp_xdesired(eqt, jdx) = 0.5*current_acceleration*delta_increment^2 + v_prev*delta_increment + x_prev;
                temp_xdesired(eqt+3, jdx) = current_acceleration*delta_increment + v_prev;
    
                x_prev = temp_xdesired(eqt, jdx);
                v_prev = temp_xdesired(eqt+3, jdx);
            end
    
        end

    else
        % Find the maximum number of increments
        n_increments_max = 0;
        for eqt = 1:3
            n_increments = ceil((dx(eqt)/wayvelocities(idx-1, eqt))/delta_increment);
            if n_increments > n_increments_max
                n_increments_max = n_increments;
            end
        end
        
        % Compute the target points for each axis
        temp_xdesired = [];
        for eqt = 1:3
            x_prev = waypoints(idx-1, eqt);
            for jdx = 1:n_increments_max
                temp_xdesired(eqt, jdx) = wayvelocities(idx-1, eqt)*delta_increment + x_prev;
                temp_xdesired(eqt+3, jdx) = wayvelocities(idx-1, eqt);
    
                x_prev = temp_xdesired(eqt, jdx);
    
            end
        end
    end
    
    if isempty(xdesired)
        xdesired = temp_xdesired;
    else
        xdesired = [xdesired, temp_xdesired];
    end
    
end


tot_increments = length(xdesired);

% Leave the drone pose as unspecified
phi = zeros(1,tot_increments);
theta = zeros(1,tot_increments);
psi = zeros(1,tot_increments);
phidot = zeros(1,tot_increments);
thetadot = zeros(1,tot_increments);
psidot = zeros(1,tot_increments);

% Populate the vector
xdesired = [xdesired; [phi;theta;psi;phidot;thetadot;psidot]];

end
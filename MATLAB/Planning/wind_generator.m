% Function to generate the matrix describing how mass changes for each timestep
function windBehaviour = wind_generator(windMode, excessWind, windStep, totalIncrements, steadyWind, windDirection)

    % Loop through all the increments required for the simulation
    for idx = 1:totalIncrements
        
        % Determine the type of wind experienced
        if windMode == "gust"
            % Generate the wind matrix for a sudden gust
            if idx < windStep 
                windBehaviour(idx, 1:3) = [steadyWind*cos(windDirection), steadyWind*sin(windDirection), 0];
            elseif idx < n_step + 1
                windBehaviour(idx, 1:3) = [(steadyWind+excessWind)*cos(windDirection), (steadyWind+excessWind)*sin(windDirection), 0];
            else
                windBehaviour(idx, 1:3) = [steadyWind*cos(windDirection), steadyWind*sin(windDirection), 0];
            end

        elseif windMode == "breeze"
            % Generate the wind matrix for a steadily increasing breeze
            if idx < windStep(1)
                windBehaviour(idx, 1:3) = [steadyWind*cos(windDirection), steadyWind*sin(windDirection), 0];
            elseif idx < windStep(2)
                windBehaviour(idx, 1:3) = [(steadyWind+excessWind/(windStep(2)-windStep(1))*(idx-windStep(1)))*cos(windDirection), (steadyWind+excessWind/(windStep(2)-windStep(1))*(idx-windStep(1)))*sin(windDirection), 0];
            else
                windBehaviour(idx, 1:3) = [(steadyWind+excessWind)*cos(windDirection), (steadyWind+excessWind)*sin(windDirection), 0];
            end
        elseif windMode == "realistic"
            % Generate the wind matrix for a randomly changing wind
            rand_angle = (rad2deg(windDirection) + 5 * randn(1, 1));
            rand_wind = steadyWind + excessWind/3 * randn(1, 1);
            windBehaviour(idx, 1:3) = [(rand_wind)*cosd(rand_angle), (rand_wind)*sind(rand_angle), 0];

        else
            % Assume no change in wind
            windBehaviour(idx, 1:3) = [steadyWind, steadyWind, steadyWind];
        end
    
    end
end
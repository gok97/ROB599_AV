% Function to generate the matrix describing how mass changes for each timestep
function massBehaviour = mass_generator(dropMode, dropMass, dropStep, totalIncrements, dryMass)

    % Loop through all the increments required for the simulation
    for idx = 1:totalIncrements
        
        % Determine the type of mass drop
        if dropMode == "delivery"
            % Generate the mass matrix for a step mass change
            if idx < dropStep
                massBehaviour(idx, 1) = dryMass(1)+dropMass;
            else
                massBehaviour(idx, 1) = dryMass(1);
            end

        elseif dropMode == "dissemination"
            % Generate the mass matrix for a ramp mass change
            if idx < dropStep(1)
                massBehaviour(idx, 1) = dryMass(1)+dropMass;
            elseif idx < dropStep(2)
                massBehaviour(idx, 1) = dryMass(1)+dropMass-(dropMass/(dropStep(2)-dropStep(1))*(idx-dropStep(1)));
            else
                massBehaviour(idx, 1) = dryMass(1);
            end

        else
            % Assume no change in mass
            massBehaviour(idx, 1) = dryMass(1);
        end
        
        % Compute the mass moments of inertia
        massBehaviour(idx, 2) = dryMass(2) + 0.00083043*(massBehaviour(idx, 1)-dryMass(1)); % THIS IS VALID ONLY FOR THE PROVIDED DRONE GEOMETRY
        massBehaviour(idx, 3) = dryMass(3) + 0.00083043*(massBehaviour(idx, 1)-dryMass(1)); % THIS IS VALID ONLY FOR THE PROVIDED DRONE GEOMETRY
        massBehaviour(idx, 4) = dryMass(4) + 0.00040757*(massBehaviour(idx, 1)-dryMass(1)); % THIS IS VALID ONLY FOR THE PROVIDED DRONE GEOMETRY
    
    end
end
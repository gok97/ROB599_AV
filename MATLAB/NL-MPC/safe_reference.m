% Function to safely read a provided trajectory
function safeReferenceTrajectory = safe_reference(startStep, finishStep, referenceTrajectory)
    
    % If the demanded trajectory goes past the end of the reference one, duplicate the last row as required
    if finishStep > length(referenceTrajectory)

        % Index the array
        safeReferenceTrajectory = referenceTrajectory(startStep:end,:);    
        currentLen = size(safeReferenceTrajectory, 1);
    
        % Add rows if necessary
        while currentLen < (finishStep-startStep+1)
            
            safeReferenceTrajectory(currentLen + 1, :) = safeReferenceTrajectory(end,:);
            currentLen = currentLen + 1;
        end

    else
        safeReferenceTrajectory = referenceTrajectory(startStep:finishStep,:);    
    end

end
%% Define the function that parses the waypoints safely
function xDesired = QuadrotorReferenceReader(start, finish, reference)

if finish > length(reference)
    % Index the array
    xDesired = reference(start:end,:);    
    current_len = size(xDesired, 1);

    % Add rows if necessary
    while current_len < (finish-start+1)
        
        xDesired(current_len + 1, :) = xDesired(end,:);
        current_len = current_len + 1;
    end
else
    xDesired = reference(start:finish,:);    
end
    %disp(xDesired)
end
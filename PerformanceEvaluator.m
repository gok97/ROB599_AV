%% Function to analyse the performance of the controller
function metrics = PerformanceEvaluator(mode, n_event, threshold, dt, xDesired, xActual)
    % Determine the type of error under assesment
    if mode == "step"
        window = 20;
    else
        window = 100;
    end

    % Find the difference between the two arrays
    delta = pdist2(xDesired(1:6, :), xActual(1:6, :));
    delta_relevant = delta(:, n_event:end);

    % Determine when the system reaches steady state
    for jdx = 3:length(delta_relevant)-2
        if delta_relevant(jdx) < threshold && delta_relevant(jdx-1) && delta_relevant(jdx-2) && delta_relevant(jdx+1) && delta_relevant(jdx+3)
            n_stable = jdx;
            break;
        end
    end
    % Compute the maximum error
    [max_error, ~] = max(delta_relevant(1:n_stable));

    % Compute the recovery time and steady state error
    recovery_time = n_stable/dt;
    ss_error = mean(delta_relevant(n_stable:n_stable+window));
    
    % Return the metrics
    metrics = [recovery_time, max_error, ss_error];
end
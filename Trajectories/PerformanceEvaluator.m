%% Function to analyse the performance of the controller
function metrics = PerformanceEvaluator(mode, n_event, dt, xDesired, xActual)
    % Determine the type of error under assesment
    if mode == "step"
        window = 20;
    else
        window = 100;
    end
    
    % Find the difference between the two arrays
    delta = pdist2(xDesired(1:6, :), xActual(1:6, :));
    delta_relevant = delta(:, n_event:end);
    
    % Compute the metrics
    [max_error, ~] = max(delta_relevant);
    [~, n_stable] = min(delta_relevant);
    recovery_time = n_stable/dt;

    ss_error = mean(delta_relevant(n_stable:n_stable+window));
    
    % Return the metrics
    metrics = [recovery_time, max_error, ss_error];
end
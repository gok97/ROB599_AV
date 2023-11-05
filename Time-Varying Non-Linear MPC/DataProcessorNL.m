%% Prepare Workspace
clear
clc

%% Data processing loop variables
pw_list = [10, 15, 20, 25, 30];
hz_list = [1, 2, 3, 4, 5];
condition = ["mass_ramp.mat", "mass_step.mat", "wind_ramp.map", "wind_step.mat"];
labels = ["Mass Ramp Input", "Mass Step Input", "Wind Ramp Input", "Wind Step Input"];
load("increment_index.mat")

for cond_idx = 1:4

    summary = [];
    error_metric = [];
    time_error_metric = [];
    counter = 1;

    for pw_idx = 1:5
    
        for hz_idx = 1:5

            % Load the relevant files
            current_fname = "NL_Cond" + condition(cond_idx) + "_Pw" + pw_list(pw_idx) + "_Hz" + hz_list(hz_idx)
            load("increment_index.mat")
            load("trajectory.mat")
            load(current_fname + "_xHistory.mat")
            load(current_fname + "_metadata.mat")
            
            % Define the time-scale
            dt = double(metadata(4));
            Ttot = (length(xDesired)-1)*dt;
            T_series = 0:dt:Ttot;
            
            % Find the difference between the two arrays
            delta = sqrt(sum((xDesired(1:6, :)' - xHistory(:, 1:6)).^2, 2));
            if cond_idx == 1 || cond_idx == 3
                delta_relevant = delta(increment_index(2):increment_index(3));
            else
                delta_relevant = delta((increment_index(1)-1):increment_index(3));
            end

            % Compute statistics
            summary(counter, 1) = mean(delta_relevant); % Average error
            summary(counter, 2) = min(delta_relevant); % Min error
            summary(counter, 3) = max(delta_relevant); % Max error
            summary(counter, 4) = metadata(1); % Compute time
            summary(counter, 5) = hz_idx; % Control Horizon
            summary(counter, 6) = pw_idx; % Preview Window

            counter = counter + 1;
        end


    end

    % Normalize the data
    max_data = max(summary(:, 1:4));
    min_data = min(summary(:, 1:4));
    normalized_data = (summary(:, 1:4) - min_data) ./ (max_data - min_data);

    % Re-arrange
    counter = 1;
    for pw_idx = 1:5
    
        for hz_idx = 1:5
            error_metric(hz_idx,pw_idx) = (normalized_data(counter, 1) + normalized_data(counter, 2) + normalized_data(counter, 3) )/3;
            time_error_metric(hz_idx, pw_idx) = (normalized_data(counter, 1) + normalized_data(counter, 2) + normalized_data(counter, 3) + 1.5*normalized_data(counter, 4))/4.5;

            counter = counter + 1;
        end

    end
    
    % Plot
    h = heatmap(time_error_metric);

    % Add axis labels and a title
    xlabel('Control Window');
    ylabel('Prediction Window');
    h.XData = hz_list;
    h.YData = pw_list;
    title(labels(cond_idx));
    pause()
    temp_var = 0;
end
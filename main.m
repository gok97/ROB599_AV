function main()
    setup_workspace();
    plot_bool = 1;
    % mass_type = "constant";
    % wind_type = "none";
    trajectory_info = load_trajectory_info(false);
    
    % extract trajectory info
    xDesired = trajectory_info{1};
    % xDesired = get_reference_trajectory(250, 0.05);
    increment_indices = trajectory_info{2};
    mass_step = trajectory_info{3};
    mass_ramp = trajectory_info{4};
    wind_ramp = trajectory_info{5};
    wind_step = trajectory_info{6};
    wind_random = trajectory_info{7};

    % [A, B, mpc_params] = initialize_params(xDesired, ones(length(mass_step), 1)*mass_step(1, :), zeros(size(wind_step)));
    [A, B, mpc_params] = initialize_params(xDesired, mass_step, wind_step);
    [Xsim, Usim] = sim_linear_mpc(A, B, mpc_params);

    if (plot_bool == 1)
        plot_mpc_traj(Xsim, mpc_params);
    end
end

function setup_workspace()
    %% Prepare workspace
    clear all
    clc
    close all

    % Load cvx optimizer
    % cvx_setup()
    cvxfile()
end

function [A, B] = discretize_and_compute_jacobians(state, input, dt, eom_params, XU0)
    % discretize continuous system
    next_state = rk4(state, input, dt, eom_params);
    tk1 = next_state(1);
    tk2 = next_state(2);
    tk3 = next_state(3);
    td1 = next_state(4);
    td2 = next_state(5);
    td3 = next_state(6);
    rk1 = next_state(7);
    rk2 = next_state(8);
    rk3 = next_state(9);
    rd1 = next_state(10);
    rd2 = next_state(11);
    rd3 = next_state(12);

    % Compute the jacobians
    [A, B] = Linearizer(tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3, XU0, false);
    A = double(A);
    B = double(B);
end

function [A, B, mpc_params] = initialize_params(xDesired, mass, wind)
    %% Set Variables
    % mass_type = "constant";
    % wind_type = "none";

    % Define the simulation interval
    dt = 0.05;

    % Define the states
    syms x y z u v w phi theta psy p q r
    state = [x y z u v w phi theta psy p q r];

    % Define the inputs
    syms w1 w2 w3 w4
    input = [w1 w2 w3 w4];

    % Define Mass
    m0 = mass(1, 1); % 0.65;
    Ix0 = mass(1, 2); % 0.0087408;
    Iy0 = mass(1, 3);
    Iz0 = mass(1, 4);
    
    % Define Wind
    w_x = wind(1, 1);
    w_y = wind(1, 2);
    w_z = wind(1, 3);

    % define simulation constants
    K = [Ix0, Iy0, Iz0, 0.01, 0.01, 0.045, 0.1, 0.1, 0.1, w_x, w_y, w_z, 0.23, 1.0, (7.5*(10^-7))/(3.13*(10^-5)), 1.0, m0, 9.81]';

    % Define the equilibrium point
    u_hover = sqrt(K(17)*K(18)/(4*K(14)));
    XU0 = [0, 0, 1.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, u_hover, u_hover, u_hover, u_hover]'; % BASIC CASE: Hover

    % construct eom parameters
    current_index = 1;
    symbolic = true;
    debug = false;
    eom_params = {K, current_index, mass, wind, symbolic, debug};
    
    % Discrerize continuous system and compute jacobians
    % z_coordinates = xDesired(:, 3);
    % for i = 1:length(z_coordinates)
    %     XU0(3) = z_coordinates(i);
    %     [A, B] = discretize_and_compute_jacobians(state, input, dt, eom_params, XU0);
    %     A_all{i} = A;
    %     B_all{i} = B;
    %     current_index = current_index + 1;
    %     eom_params{2} = current_index;
    %     disp(i)
    % end
    % current_index = 1;
    % eom_params{2} = current_index;

    [A, B] = discretize_and_compute_jacobians(state, input, dt, eom_params, XU0);

    % Define the MPC parameters
    nx = 12;
    nu = 4;
    N = 250;
    horizon = 40;
    X0 = [0, 0, 1.2, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    Xbar = X0;
    Ubar = [u_hover, u_hover, u_hover, u_hover];
    Q = 10*eye(nx);
    R = 0.1*eye(nu);

    % Define the reference trajectory
    % Xref = get_reference_trajectory(N, dt);
    Xref = xDesired;
    
    % Define the reference control input
    Uref = ones(N-1, nu)*u_hover;

    % Define the parameters for the MPC Problem
    eom_params{5} = false; % set symbolic flag to false
    mpc_params = {horizon, Q, R, Xbar, Ubar, Xref, Uref, nx, nu, dt, eom_params};

end

function Xref = get_reference_trajectory(N, dt)
    Xref = [];
    i = 1;
    for t = linspace(-pi/2, 3*pi/2 + 4*pi, N)
        Xref(i, :) = [5*cos(t), 5*cos(t)*sin(t), 1.2, zeros(1, 9)];
        i = i +1;
    end

    for i = 1:(N-1)
        Xref(i,4:6) = (Xref(i+1,1:3) - Xref(i, 1:3))/dt;
    end
end

function trajectory_info = load_trajectory_info(plot_bool)
    % load trajectory info
    xDesired = load("Trajectories\trajectory.mat").xDesired';
    % load increment indices info
    increment_index = load("Trajectories\increment_index.mat").n_summary;
    % load mass info 
    mass_step = load("Trajectories\mass_step.mat").mass_matrix;
    mass_ramp = load("Trajectories\mass_ramp.mat").mass_matrix;
    % load wind info
    wind_ramp = load("Trajectories\wind_ramp.mat").wind_matrix;
    wind_step = load("Trajectories\wind_step.mat").wind_matrix;
    wind_random = load("Trajectories\wind_random.mat").wind_matrix;
    
    trajectory_info = {xDesired, increment_index, mass_step, mass_ramp, wind_ramp, wind_step, wind_random};
    plot_data = {xDesired(1, :), xDesired(2, :), xDesired(3, :), xDesired(4, :), xDesired(5, :), xDesired(6, :), wind_step(:, 1), wind_step(:, 2), mass_step(:, 1), wind_ramp(:, 1), wind_ramp(:, 2), mass_ramp(:, 1)};
    tot_inc = 0.01;
    T_series = 0:tot_inc:(length(plot_data{1})-1)*tot_inc;
    
    if plot_bool
        labels = ["X (m)", "Y (m)", "Z (m)", "Xdot (m/s)", "Ydot (m/s)", "Zdot (m/s)", "X Wind (m/s)", "Y Wind (m/s)", "Mass (Kg)", "X Wind (m/s)", "Y Wind (m/s)", "Mass (Kg)"];        
        for i = 1:12
            % Create a subplot in the ith position
            subplot(4, 3, i);
        
            % Create the line plot for the target and actual cases
            plot(T_series, plot_data{i});
        
            % Add titles or labels as needed
            xlabel('Time (s)');
            ylabel(labels(i));
        end
        hold off
    end
end

function plot_mpc_traj(Xsim, mpc_params)  
    mode = 0;
    dt = mpc_params{10};
    K = mpc_params{11}{1};
    % Plot animation
    animation_fig = figure(2);
    labels = ["x", "y", "z"];
    xlabel(labels(1));
    ylabel(labels(2));
    zlabel(labels(3));
    trajectory_line_nl = animatedline('MaximumNumPoints',10000, 'Color','yellow');
    hold on
    
    drone_shape = [ K(13)/sqrt(2),  0,              -K(13)/sqrt(2), K(13)/sqrt(2),  0,              -K(13)/sqrt(2);       
                    -K(13)/sqrt(2), 0,              -K(13)/sqrt(2), K(13)/sqrt(2),  0,              K(13)/sqrt(2);
                    0,              0,              0,              0,              0,              0;
                    1,              1,              1,              1,              1,              1               ];   
    
    % % Prepare video object
    % writerObj = VideoWriter('straight_video.avi', 'Motion JPEG AVI');
    % writerObj.Quality = 90;
    % writerObj.FrameRate = 100; % Adjust the frame rate as needed
    % open(writerObj);

    for i = 1:length(Xsim)
        % Compute values for the NL model
        % Compute and apply the rotation transformation
        angles_nl = Xsim(i, 7:9);
        R_nl = eul2rotm(angles_nl,'ZYX');
    
        % Compute the translation
        t_vector_nl = [Xsim(i, 1:3)];
    
        % Form a homogeneous transformation matrix
        H_nl = [R_nl, t_vector_nl'; 0, 0, 0, 1];
        new_drone_shape_nl = H_nl*drone_shape;
    
        % Display
        if i == 1
            p = plot3(new_drone_shape_nl(1,1:6),new_drone_shape_nl(2,1:6),new_drone_shape_nl(3,1:6),'b.-');
        else
            set(p, 'XData', new_drone_shape_nl(1,1:6), 'YData', new_drone_shape_nl(2,1:6),  'ZData', new_drone_shape_nl(3,1:6));
        end
        addpoints(trajectory_line_nl, Xsim(i,1), Xsim(i,2), Xsim(i,3));
    
        % Adjust view and set limits
        view(30, 45);
        legend("NL Drone", "L Drone", 'Position', [0.1, 0.05, 0.2, 0.05])
        switch mode
            case 0
                xlim([Xsim(i, 1)-3, Xsim(i, 1)+3])
                ylim([Xsim(i, 2)-3, Xsim(i, 2)+3])
                zlim([Xsim(i, 3)-3, Xsim(i, 3)+3])
            case 1
                limiting_min_axis = min(min(Xsim(:, 1:3)));
                limiting_max_axis = max(max(Xsim(:, 1:3)));
                delta = limiting_max_axis - limiting_min_axis;
                avg_x = (mean(Xsim(:, 1)));
                avg_y = (mean(Xsim(:, 2)));
                avg_z = (mean(Xsim(:, 3)));
                xlim([min(avg_x) - delta/2 - 10, max(avg_x) + delta/2 + 10])
                ylim([min(avg_y) - delta/2 - 10, max(avg_y) + delta/2 + 10])
                zlim([min(avg_z) - delta/2 - 10, max(avg_z) + delta/2 + 10])
        end
        

        % Capture the current frame
        frame = getframe(animation_fig);
        
        % % Write the frame to the video
        % writeVideo(writerObj, frame);

        % Fix framerate to match rate of sampling and update plot
        pause(dt);
        drawnow;
    end

    % % Save the video
    % close(writerObj);
end

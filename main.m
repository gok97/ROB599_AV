function main()
    setup_workspace();
    plot_bool = 1;
    mass_type = "constant";
    wind_type = "none";
    [A, B, mpc_params] = initialize_params(mass_type, wind_type);
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

function [A, B, mpc_params] = initialize_params(mass_type, wind_type)
    %% Set Variables
    mass_type = "constant";
    wind_type = "none";
    current_time = 0;

    % Define the simulation interval
    dt = 0.1;

    % Define the states
    syms x y z u v w phi theta psy p q r
    state = [x y z u v w phi theta psy p q r];

    % Define the inputs
    syms w1 w2 w3 w4
    input = [w1 w2 w3 w4];

    % Define Mass
    m0 = 0.65;
    Ix0 = 0.0087408;
    Iy0 = 0.0087408;
    Iz0 = 0.0173188;

    switch mass_type
       case "constant"
           m = m0;
           Ix = Ix0;
           Iy = Iy0;
           Iz = Iz0;
           
       case "symbolic"
           syms Ix Iy Iz m
    end

    % Define Wind
    switch wind_type
        case "none"
            w_x = 0;
            w_y = 0;
            w_z = 0;

        case "symbolic"
           syms w_x w_y w_z
    end

    % define simulation constants
    K = [Ix0, Iy0, Iz0, 0.01, 0.01, 0.045, 0.1, 0.1, 0.1, w_x, w_y, w_z, 0.23, 1.0, (7.5*(10^-7))/(3.13*(10^-5)), 1.0, m0, 9.81]';

    % Define the equilibrium point
    u_hover = sqrt(K(17)*K(18)/(4*K(14)));
    XU0 = [0, 0, 1.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, u_hover, u_hover, u_hover, u_hover]'; % BASIC CASE: Hover

    % construct eom parameters
    symbolic = true;
    debug = false;
    eom_params = {K, current_time, mass_type, wind_type, symbolic, debug};

    % Discrerize continuous system and compute jacobians
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
    Xref = get_reference_trajectory(N, dt);
    
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

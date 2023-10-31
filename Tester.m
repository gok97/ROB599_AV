%% Prepare workspace
clear
clc
close all

% Load cvx optimizer
cvxfile()

%% Set Variables
condition = "hover";
mass = "constant";
wind = "constant";
mode = 0;
plot_bool = 1;
study = false;

% Define the simulation interval
syms T
t0 = 0;
t1 = 4;
dt = 0.01;

% Define Mass
m0 = 0.65;
Ix0 = 0.0087408;
Iy0 = 0.0087408;
Iz0 = 0.0173188;

switch mass
    case "constant"
        m = m0;
        Ix = Ix0;
        Iy = Iy0;
        Iz = Iz0;

    case "continuous"
        mass_rate = 0.05;
        m = m0 + (mass_rate*(t1-t0)) - mass_rate*T;
        Ix = Ix0 + 0.00040757*(m-m0);
        Iy = Iy0 + 0.00040757*(m-m0);
        Iz = Iz0 + 0.00040757*(m-m0);

    case "discrete"
        delta_mass = m0*0.25;
        t_drop = (t1+t0)/2;
        m = m0+ delta_mass -(1/(1+exp(-10000*(T-t_drop)))*delta_mass);
        Ix = Ix0 + 0.00040757*(m-m0);
        Iy = Iy0 + 0.00040757*(m-m0);
        Iz = Iz0 + 0.00040757*(m-m0);
        
    case "symbolic"
        syms Ix Iy Iz m

end

% Define Wind
switch wind
    case "none"
        w_x = 0;
        w_y = 0;
        w_z = 0;

    case "constant"
        w_x = 0;
        w_y = 0;
        w_z = 0;

    case "continuos+"
        w_x = 0;
        w_y = 0;
        w_z = 0;

    case "continuos-"
        w_x = 0;
        w_y = 0;
        w_z = 0;

    case "gust+"
        w_x = 0;
        w_y = 0;
        w_z = 0;

    case "gust-"
        w_x = 0;
        w_y = 0;
        w_z = 0;

    case "symbolic"
        syms w_x w_y w_z

end

% Define simulation constants
% K = [Ix, Iy, Iz, Ax, Ay, Az, kdx, kdy, kdz, xdot_w, ydot_w, zdot_w, l, kf, km, ka, m, g];
% K = [Ix, Iy, Iz, 0.01, 0.01, 0.045, 0.1, 0.1, 0.1, w_x, w_y, w_z, 0.23, 3.13*(10^-5), 7.5*(10^-7), 1.0, m, 9.81]';
K = [Ix, Iy, Iz, 0.01, 0.01, 0.045, 0.1, 0.1, 0.1, w_x, w_y, w_z, 0.23, 1.0, 7.5*(10^-7), 1.0, m, 9.81]';

% Define Initial Condition
switch condition
    case "hover"
        u_hover = sqrt(K(17)*K(18)/(4*K(14)));
        X0 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

    case "ascent"
        ws_ascent = 1;
        u_ascent = sqrt((K(17)*K(18)+(K(16)*K(6)*(1^2)))/(4*K(14)));
        X0 = [0, 0, 0, 0, 0, ws_ascent, 0, 0, 0, 0, 0, 0];

    case "forward"
        syms us_straight ws_straight theta_straight u_straight
        forward_speed = 2;
        % NEEDS REVIEWING
        eqA = 4*K(14)*u_straight^2 == K(18)*K(17)*cos(theta_straight) + K(16)*K(6)*ws_straight^2;
        eqB = K(18)*K(17)*sin(theta_straight) == K(16)*K(4)*us_straight^2;
        eqC = 2*sin(theta_straight) == ws_straight;
        eqD = 2*cos(theta_straight) == us_straight;
        [us_straight, ws_straight, theta_straight, u_straight] = solve([eqA, eqB, eqC, eqD], [us_straight, ws_straight, theta_straight, u_straight]);
        us_straight = double(us_straight);
        ws_straight = double(ws_straight);
        theta_straight = double(theta_straight);
        u_straight = double(u_straight);
        X0 = [0, 0, 0, us_straight(1), 0, ws_straight(1), 0, theta_straight, 0, 0, 0, 0];

    case "diagonal"
end

% Define the equilibrium point
% XU0 = [x, y, z, u, v, w, phi, theta, psy, p, q, r, w1, w2, w3, w4];
XU0 = [0, 0, 1.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, u_hover, u_hover, u_hover, u_hover]'; % BASIC CASE: Hover

%% Compute the nonlinear equations and their linearized counterparts
% Substitute for constants
[tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3] = EquationsOfMotion(K, false, true);

% Compute the jacobians
[A, B] = Linearizer(tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3, XU0, true);

% Prepare the function handles for the equations of motion
[tk1_function, tk2_function, tk3_function, td1_function, td2_function, td3_function, rk1_function, rk2_function, rk3_function, rd1_function, rd2_function, rd3_function] = Nonlinearizer(tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3);

% Solve ODE for nonlinear equations
NonlinearizedDroneFunction = @(Tn, Xn) NonlinearizedDrone(Tn, Xn, tk1_function, tk2_function, tk3_function, td1_function, td2_function, td3_function, rk1_function, rk2_function, rk3_function, rd1_function, rd2_function, rd3_function);
[T_nl,X_nl] = ode45(NonlinearizedDroneFunction, t0: dt: t1, X0);

% Solve ODE for linear equations
LinearizedDroneFunction = @(Tl, Xl) LinearizedDrone(Tl, Xl, A, B);
[T_l,X_l] = ode45(LinearizedDroneFunction,t0: dt: t1, X0);

%% Study stability
% Compute the eigenvectors and eigenvalues
if study == true
    [e_vector, e_value] = eig(A);
    e_value = diag(e_value);
    
    disp("Eigenvalues: ")
    disp(e_value);
    
    disp("Eigenvectors: ")
    disp(e_vector);
    
    % Additionally, compute jordan form
    [e_vector_matrix, Abar] = jordan(A);
end

%% Study Controllability
% Loop through all the eigenvalues and perform the PBH test
if study == true
    counter_controllable = 0;
    
    for i = 1:12
        current_rank = rank(double([(A-(Abar(i,i)*eye(12))), B]), 1e-6);
        if current_rank == 12
            disp("Controllable Eigenvalue:");
            disp(Abar(i, i));
            disp("Associated Eigenvector: ");
            disp(e_vector_matrix(:, i))
            counter_controllable = counter_controllable + 1;
        end
    end
    
    disp("Total Controllable Modes: ")
    disp(counter_controllable)
end

%% Plot results
% Only plot if desired
if plot_bool == 1
    % Plot all state variables against time
    summary_fig = figure(1);
    labels = ["X (m)", "Y (m)", "Z (m)", "U (m/s)", "V (m/s)", "W (m/s)", "PHI (rad)", "THETA (rad)", "PSI (rad)", "P (rad/s)", "Q (rad/s)", "R (rad/s)"];
    
    for i = 1:12
        % Create a subplot in the ith position
        subplot(4, 3, i);
    
        % Create the line plot for the nonlinear and linear cases
        plot(T_nl, X_nl(:, i));
        hold on;
        plot(T_l, X_l(:, i));
        if  i == 6
            ylim([0,2])

        end
        % Add titles or labels as needed
        xlabel('Time (s)');
        ylabel(labels(i));

    end
    sgtitle('Comparison of NL and L models');
    legend("NL Model", "L Model", 'Position', [0.425, 0.005, 0.2, 0.05])  
    legend boxoff  
    hold off
    
    
    % Plot animation
    animation_fig = figure(2);
    xlabel(labels(1));
    ylabel(labels(2));
    zlabel(labels(3));
    trajectory_line_nl = animatedline('MaximumNumPoints',10000, 'Color','cyan');
    trajectory_line_l = animatedline('MaximumNumPoints',10000, 'Color','yellow');
    hold on
    
    drone_shape = [ K(13)/sqrt(2),  0,              -K(13)/sqrt(2), K(13)/sqrt(2),  0,              -K(13)/sqrt(2);       
                    -K(13)/sqrt(2), 0,              -K(13)/sqrt(2), K(13)/sqrt(2),  0,              K(13)/sqrt(2);
                    0,              0,              0,              0,              0,              0;
                    1,              1,              1,              1,              1,              1               ];   
    

    writerObj = VideoWriter('straight_video.avi', 'Motion JPEG AVI');
    writerObj.Quality = 90;
    writerObj.FrameRate = 100; % Adjust the frame rate as needed
    open(writerObj);

    for i = 1:length(T_nl)
        % Compute values for the NL model
        % Compute and apply the rotation transformation
        angles_nl = X_nl(i, 7:9);
        R_nl = eul2rotm(angles_nl,'ZYX');
    
        % Compute the translation
        t_vector_nl = [X_nl(i, 1:3)];
    
        % Form a homogeneous transformation matrix
        H_nl = [R_nl, t_vector_nl'; 0, 0, 0, 1];
        new_drone_shape_nl = H_nl*drone_shape;
        
        % Compute values for the L model
        % Compute and apply the rotation transformation
        angles_l = X_l(i, 7:9);
        R_l = eul2rotm(angles_l,'ZYX');
    
        % Compute the translation
        t_vector_l = [X_l(i, 1:3)];
    
        % Form a homogeneous transformation matrix
        H_l = [R_l, t_vector_l'; 0, 0, 0, 1];
        new_drone_shape_l = H_l*drone_shape;
    
        % Display
        title("Drone Animation - Time: " + num2str(T_nl(i)));
        if i == 1
            p = plot3(new_drone_shape_nl(1,1:6),new_drone_shape_nl(2,1:6),new_drone_shape_nl(3,1:6),'b.-');
            q = plot3(new_drone_shape_l(1,1:6),new_drone_shape_l(2,1:6),new_drone_shape_l(3,1:6),'r.-');
        else
            set(p, 'XData', new_drone_shape_nl(1,1:6), 'YData', new_drone_shape_nl(2,1:6),  'ZData', new_drone_shape_nl(3,1:6));
            set(q, 'XData', new_drone_shape_l(1,1:6), 'YData', new_drone_shape_l(2,1:6),  'ZData', new_drone_shape_l(3,1:6));  
        end
        addpoints(trajectory_line_nl,X_nl(i,1),X_nl(i,2),X_nl(i,3));
        addpoints(trajectory_line_l,X_l(i,1),X_l(i,2),X_l(i,3));
    
        % Adjust view and set limits
        view(30, 45);
        legend("NL Drone", "L Drone", 'Position', [0.1, 0.05, 0.2, 0.05])
        switch mode
            case 0
                xlim([min(X_nl(i, 1)-3,X_l(i, 1)-3), max(X_nl(i, 1)+3, X_l(i, 1)+3)])
                ylim([min(X_nl(i, 2)-3,X_l(i, 2)-3), max(X_nl(i, 2)+3, X_l(i, 2)+3)])
                zlim([min(X_nl(i, 3)-3,X_l(i, 3)-3), max(X_nl(i, 3)+3, X_l(i, 3)+3)])
            case 1
                limiting_min_axis = min(min(min(X_nl(:, 1:3)), min(X_l(:, 1:3))));
                limiting_max_axis = max(max(max(X_nl(:, 1:3)), max(X_l(:, 1:3))));
                delta = limiting_max_axis - limiting_min_axis;
                avg_x = (mean([X_nl(:, 1), X_l(:, 1)]));
                avg_y = (mean([X_nl(:, 2), X_l(:, 2)]));
                avg_z = (mean([X_nl(:, 3), X_l(:, 3)]));
                xlim([min(avg_x) - delta/2 - 10, max(avg_x) + delta/2 + 10])
                ylim([min(avg_y) - delta/2 - 10, max(avg_y) + delta/2 + 10])
                zlim([min(avg_z) - delta/2 - 10, max(avg_z) + delta/2 + 10])
        end
        

        % Capture the current frame
        frame = getframe(animation_fig);
        
        % Write the frame to the video
        writeVideo(writerObj, frame);

        % Fix framerate to match rate of sampling and update plot
        pause(dt);
        drawnow;
    end

    % Save the video
    close(writerObj);

end

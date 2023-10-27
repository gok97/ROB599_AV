%% Prepare workspace
clear all
clc
close all

% Load cvx optimizer
cvxfile()

%% Set Variables
condition = "hover";
mass = "constant";
wind = "none";
mode = 0;
plot_bool = 1;
study = false;

% MPC Parameters
nx = 12;
nu = 4;
N = 250;
horizon = 40;

% Define the simulation interval
syms T
t0 = 0;
t1 = 4;
dt = 0.1;

% Define the states
syms x y z u v w phi theta psy p q r 

% Define the inputs
syms w1 w2 w3 w4


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
        
    case "symbolic"
        syms Ix Iy Iz m

end

% Define Wind
switch wind
    case "none"
        w_x = 0;
        w_y = 0;
        w_z = 0;

    case "symbolic"
        syms w_x w_y w_z

end

% Define simulation constants
% K = [Ix, Iy, Iz, Ax, Ay, Az, kdx, kdy, kdz, xdot_w, ydot_w, zdot_w, l, kf, km, ka, m, g];
K = [Ix, Iy, Iz, 0.01, 0.01, 0.045, 0.1, 0.1, 0.1, w_x, w_y, w_z, 0.23, 3.13*(10^-5), 7.5*(10^-7), 1.0, m, 9.81]';

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
XU0 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, u_hover, u_hover, u_hover, u_hover]'; % BASIC CASE: Hover

%% Compute the nonlinear equations and their linearized counterparts
% Substitute for constants
[tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3] = EquationsOfMotion(K, true, true);

% Compute the jacobians
[A, B] = Linearizer(tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3, XU0, false);

% Prepare the function handles for the equations of motion
[tk1_function, tk2_function, tk3_function, td1_function, td2_function, td3_function, rk1_function, rk2_function, rk3_function, rd1_function, rd2_function, rd3_function] = Nonlinearizer(tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3);


%% Solve MPC
% Define the MPC parameters
X0 = [0, 0, 1.2, 0, 0, 0, 0, 0, 0, 0, 0, 0];
Xbar = X0;
Ubar = [u_hover, u_hover, u_hover, u_hover];
Q = 10*eye(nx);
R = 0.1*eye(nu);

% Define the reference trajectory
Xref = [];
i = 1;
%for t = linspace(0, 5, N)
for t = linspace(-pi/2, 3*pi/2 + 4*pi, N)
    Xref(i, :) = [5*cos(t), 5*cos(t)*sin(t), 1.2, zeros(1, 9)];
    %Xref(i, :) = [t, 0, 1.2, zeros(1, 9)];
    i = i +1;
end

for i = 1:(N-1)
    Xref(i,4:6) = (Xref(i+1,1:3) - Xref(i, 1:3))/dt;
end

% Define the reference control input
Uref = ones(N-1, nu)*u_hover;

% Define the parameters for the MPC Problem
% parameters = [horizon, Q, R, Xbar, Ubar, Xref, Uref, nx, nu, dt]
parameters = {horizon, Q, R, Xbar, Ubar, Xref, Uref, nx, nu, dt};

% Execute the simulation
N_sim = 100;
Xsim = [];
Usim = [];
for i = 1:N_sim
    Xsim(i, :) = X0;
end
for i = 1:N_sim-1
    Usim(i, :) = [0, 0, 0, 0];
end

eom_list = stacker(tk1, tk2, tk3, td1, td2, td3, rk1, rk2, rk3, rd1, rd2, rd3);
eom_list = matlabFunction(eom_list, 'Vars',  [x, y, z, u, v, w, phi, theta, psy, p, q, r, w1, w2, w3, w4]);

for i = 1:(N_sim-1)
    fprintf("simulation iteration: %d", i);
    Usim(i, :) = DroneMPC(A, B, eom_list, parameters, X0, i);
    % Calculate the next step of the simulation
    [x_out, y_out, z_out, u_out, v_out, w_out, phi_out, theta_out, psy_out, p_out, q_out, r_out, w1_out, w2_out, w3_out, w4_out] = parser(Xsim(i, :), Usim(i, :));
    Xsim(i+1, :) = eom_list(x_out, y_out, z_out, u_out, v_out, w_out, phi_out, theta_out, psy_out, p_out, q_out, r_out, w1_out, w2_out, w3_out, w4_out);

end

%% Plot results
% Only plot if desired
if plot_bool == 1
      
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

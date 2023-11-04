%% Prepare Workspace
clear
clc

%% Set the problem variables

% Set simulation parameters
dt = 0.1; % Time increment
pw = 18; % Prediction window
hz = 2; % Horizon

MV_penalty = 0.1; % Penalise the controls tracking in favour of the output tracking
MVrate_penallty = 0.1; % Penalty excessive control changes

% Set visualization settings
video_name = 'test_video.avi';
video_mode = "follow";

% Disturbance settings
mass = "constant";
wind = "none";

xdot_w = 0;
ydot_w = 0;
zdot_w = 0;

% Generate the reference trajectory
wp=[0,0,0;
    0,0,1;
    5,5,1;
    5,5,0.1];
wv=[0,0,0;
    0,0,0;
    0,0,0;
    0,0,0];

xTarget = QuadrotorRawTrajectoryImproved(dt, wp, wv)';

Ttot = (length(xTarget)*dt)-dt;
T_series = 0:dt:Ttot;

%% Compute time dependent disturbances such as mass and wind velocity
syms t_m t_w
t1 = 12.5; % End time of continuous drop
t0 = 7.5; % Start Time of continuous drop
t_drop = 10; % Time of descrete drop

tw1 = 12.5; % End time of continuous drop
tw0 = 7.5; % Start Time of continuous drop
tw_gust = 10; % Time of descrete drop

% Define system mass w/o payload
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
        m = m0 + (mass_rate*(t1-t0)) - mass_rate*(t_m-t0);
        Ix = Ix0 + 0.00040757*(m-m0);
        Iy = Iy0 + 0.00040757*(m-m0);
        Iz = Iz0 + 0.00040757*(m-m0);

    case "discrete"
        delta_mass = m0*0.75;
        m = m0+ delta_mass -(1/(1+exp(-10000*(t_m-t_drop)))*delta_mass);
        Ix = Ix0 + 0.00040757*(m-m0);
        Iy = Iy0 + 0.00040757*(m-m0);
        Iz = Iz0 + 0.00040757*(m-m0);

end

% Define Wind
w_base = 4;
switch wind
    case "none"
        w_x = 0;
        w_y = 0;
        w_z = 0;

    case "constant"
        w_tot = w_base;
        w_x = w_tot*sqrt(2);
        w_y = w_tot*sqrt(2);
        w_z = 0;

    case "continuos+"
        w_rate = 0.05;
        w_tot = w_base + w_rate*(t_w-tw0);
        w_x = w_tot*sqrt(2);
        w_y = w_tot*sqrt(2);
        w_z = 0;

    case "continuos-"
        w_rate = 0.05;
        w_tot = w_base - w_rate*(t_w-tw0);
        w_x = w_tot*sqrt(2);
        w_y = w_tot*sqrt(2);
        w_z = 0;

    case "gust+"
        delta_w = w_base;
        w_tot = w_base + delta_w -(1/(1+exp(-10000*(t_w-tw_gust)))*delta_w);
        w_x = w_tot*sqrt(2);
        w_y = w_tot*sqrt(2);
        w_z = 0;

    case "gust-"
        delta_w = -w_base;
        w_tot = w_base + delta_w -(1/(1+exp(-10000*(t_w-tw_gust)))*delta_w); %% NOT SURE IF CORRECT
        w_x = w_tot*sqrt(2);
        w_y = w_tot*sqrt(2);
        w_z = 0;

end

% Substitute in the constants array
% constants = [Ix, Iy, Iz, Ax, Ay, Az, kdx, kdy, kdz, xdot_w, ydot_w, zdot_w, l, kf, km, ka, m, g];
% constants = [Ix, Iy, Iz, 0.01, 0.01, 0.045, 0.1, 0.1, 0.1, xdot_w, ydot_w, zdot_w, 0.23, 3.13*(10^-5), 7.5*(10^-7), 1.0, m, 9.81]';
constants = [Ix, Iy, Iz, 0.01, 0.01, 0.045, 0.1, 0.1, 0.1, w_x, w_y, w_z, 0.23, 1, 7.5*(10^-7)/(3.13*(10^-5)), 1.0, m, 9.81]';

% Compute an approximate hover condition
% u_hover = sqrt(constants(17)*constants(18)/(4*constants(14)));
u_hover = double(sqrt(m0*constants(18)/(4*constants(14))));

%% Execute the simulation
% Specify the initial conditions
x = [0;0;0;0;0;0;0;0;0;0;0;0];

% Define a nominal control target to maintain the quadcopter hovering
nloptions = nlmpcmoveopt;
nloptions.MVTarget = [u_hover u_hover u_hover u_hover];
mv = nloptions.MVTarget;

% Display waitbar to show simulation progress
hbar = waitbar(0,"Simulation Progress");

% MV last value is part of the controller state
lastMV = mv;

% Store states for plotting purposes
xHistory = x';
uHistory = lastMV;


% Simulation loop
tic
for k = 1:(Ttot/dt)

    % Set references for previewing
    t = linspace(k*dt, (k+pw-1)*dt,pw);

    % yref = QuadrotorReferenceTrajectory(t);
    yref = QuadrotorReferenceReader(k, k+pw-1, xTarget);
    
    
    % PREPARE THE NEW MODEL
        Ix_current = double(subs(Ix, t_m, t(1)));
        Iy_current = double(subs(Iy, t_m, t(1)));
        Iz_current = double(subs(Iz, t_m, t(1)));
        m_current = double(subs(m, t_m, t(1)));
        xdot_w_current = double(subs(w_x, t_w, t(1)));
        ydot_w_current = double(subs(w_y, t_w, t(1)));
        zdot_w_current = double(subs(w_z, t_w, t(1)));

        mHistory(k) = m_current;
        wHistory(k, :) = [xdot_w_current, ydot_w_current, zdot_w_current];

        constants = [Ix_current, Iy_current, Iz_current, 0.01, 0.01, 0.045, 0.1, 0.1, 0.1, xdot_w_current, ydot_w_current, zdot_w_current, 0.23, 1, 7.5*(10^-7), 1.0, m_current, 9.81]';

        QuadcopterModel;

        % Define the non-linear mpc problem to feature 12 states, 12 outputs, and 4 inputs
        nx = 12;
        ny = 12;
        nu = 4;
        nlmpcobj = nlmpc(nx, ny, nu);
        
        % Associate the state and the jacobian functions with the problem:
        nlmpcobj.Model.StateFcn = "QuadrotorStateFcn";
        nlmpcobj.Jacobian.StateFcn = @QuadrotorStateJacobianFcn;
        rng(0)
        validateFcns(nlmpcobj,rand(nx,1),rand(nu,1));
        
        % Specify the MPC parameters
        nlmpcobj.Ts = dt;
        nlmpcobj.PredictionHorizon = pw;
        nlmpcobj.ControlHorizon = hz;
        
        % Add constraints to the control inputs:
        nlmpcobj.MV = struct( ...
            Min={0;0;0;0}, ...
            Max={u_hover*5;u_hover*5;u_hover*5;u_hover*5}, ...
            RateMin={-u_hover*5;-u_hover*5;-u_hover*5;-u_hover*5}, ...
            RateMax={u_hover*2.5;u_hover*2.5;u_hover*2.5;u_hover*2.5} ...
            );
        
        % Define the weights for the non-linear MPC
        nlmpcobj.Weights.OutputVariables = [1 1 1 0 0 0 1 1 1 0 0 0]; % Define the output variables to be tracked
        nlmpcobj.Weights.ManipulatedVariables = [MV_penalty MV_penalty MV_penalty MV_penalty];
        nlmpcobj.Weights.ManipulatedVariablesRate = [MVrate_penallty MVrate_penallty MVrate_penallty MVrate_penallty];


    % END OF NEW MODEL PREPARATION


    % Compute control move with reference previewing
    xk = xHistory(k,:);
    [uk,nloptions,info] = nlmpcmove(nlmpcobj,xk,double(lastMV),yref,[],nloptions);

    % Store control move
    uHistory(k+1,:) = uk';
    lastMV = uk;

    % Simulate quadrotor for the next control interval (MVs = uk) 
    ODEFUN = @(t,xk) QuadrotorStateFcn(xk,uk);
    [TOUT,XOUT] = ode45(ODEFUN,[0 dt], xHistory(k,:)');

    % Update quadrotor state
    xHistory(k+1,:) = XOUT(end,:);

    % Update waitbar
    waitbar(k*dt/Ttot,hbar);
end
toc
% Close waitbar 
close(hbar)

%% Visualize the results
% Plot all state variables against time
summary_fig = figure(1);
labels = ["X (m)", "Y (m)", "Z (m)", "Xdot (m/s)", "Ydot (m/s)", "Zdot (m/s)", "PHI (rad)", "THETA (rad)", "PSI (rad)", "P (rad/s)", "Q (rad/s)", "R (rad/s)"];

% T_series = 0:dt:Ttot;
% xTarget = QuadrotorReferenceTrajectory(T_series)';

for i = 1:6
    % Create a subplot in the ith position
    subplot(2, 3, i);

    % Create the line plot for the target and actual cases
    plot(T_series, xHistory(:, i));
    hold on;
    plot(T_series, xTarget(:, i));

    % Add titles or labels as needed
    xlabel('Time (s)');
    ylabel(labels(i));

end
sgtitle('Comparison of Target and Actual trajectory');
legend("Actual", "Target", 'Position', [0.425, 0.005, 0.2, 0.05])  
legend boxoff  
hold off

pause()

% Plot animation
animation_fig = figure(2);
xlabel(labels(1));
ylabel(labels(2));
zlabel(labels(3));
trajectory_line_nl = animatedline('MaximumNumPoints',10000, 'Color','cyan');
trajectory_line_l = animatedline('MaximumNumPoints',10000, 'Color','yellow');
hold on

drone_shape = [ constants(13)/sqrt(2),  0,              -constants(13)/sqrt(2), constants(13)/sqrt(2),  0,              -constants(13)/sqrt(2);       
                -constants(13)/sqrt(2), 0,              -constants(13)/sqrt(2), constants(13)/sqrt(2),  0,              constants(13)/sqrt(2);
                0,                      0,              0,                      0,                      0,              0;
                1,                      1,              1,                      1,                      1,              1               ];   


writerObj = VideoWriter(video_name, 'Motion JPEG AVI');
writerObj.Quality = 90;
writerObj.FrameRate = 100; % Adjust the frame rate as needed
open(writerObj);

for i = 1:length(T_series)
    % Compute values for the Actual
    % Compute and apply the rotation transformation
    angles_nl = xHistory(i, 7:9);
    R_nl = eul2rotm(angles_nl,'ZYX');

    % Compute the translation
    t_vector_nl = [xHistory(i, 1:3)];

    % Form a homogeneous transformation matrix
    H_nl = [R_nl, t_vector_nl'; 0, 0, 0, 1];
    new_drone_shape_nl = H_nl*drone_shape;

    % Display
    title("Drone Animation - Time: " + num2str(T_series(i)));
    if i == 1
        p = plot3(new_drone_shape_nl(1,1:6),new_drone_shape_nl(2,1:6),new_drone_shape_nl(3,1:6),'b.-');
        
        % Draw a trajectory for the Target
        q = plot3(xTarget(:,1), xTarget(:,2), xTarget(:,3),'r.-');

    else
        set(p, 'XData', new_drone_shape_nl(1,1:6), 'YData', new_drone_shape_nl(2,1:6),  'ZData', new_drone_shape_nl(3,1:6));
    end
    addpoints(trajectory_line_nl,xHistory(i,1),xHistory(i,2),xHistory(i,3));

    % Adjust view and set limits
    view(30, 45);
    legend("Actual", "Target", 'Position', [0.1, 0.05, 0.2, 0.05])
    switch video_mode
        case "follow"
            xlim([min(xHistory(i, 1)-3,xTarget(i, 1)-3), max(xHistory(i, 1)+3, xTarget(i, 1)+3)])
            ylim([min(xHistory(i, 2)-3,xTarget(i, 2)-3), max(xHistory(i, 2)+3, xTarget(i, 2)+3)])
            zlim([min(xHistory(i, 3)-3,xTarget(i, 3)-3), max(xHistory(i, 3)+3, xTarget(i, 3)+3)])
        case "global"
            limiting_min_axis = min(min(min(xHistory(:, 1:3)), min(xTarget(:, 1:3))));
            limiting_max_axis = max(max(max(xHistory(:, 1:3)), max(xTarget(:, 1:3))));
            delta = limiting_max_axis - limiting_min_axis;
            avg_x = (mean([xHistory(:, 1), xTarget(:, 1)]));
            avg_y = (mean([xHistory(:, 2), xTarget(:, 2)]));
            avg_z = (mean([xHistory(:, 3), xTarget(:, 3)]));
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


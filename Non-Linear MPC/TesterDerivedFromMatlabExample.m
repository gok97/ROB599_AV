%% Prepare Workspace
clear
clc

%% Set the problem variables
dt = 0.1; % Time increment
pw = 18; % Prediction window
hz = 2; % Horizon

MV_penalty = 0.1; % Penalise the controls tracking in favour of the output tracking
MVrate_penallty = 0.1; % Penalty excessive control changes

Ttot = 20; % Simulation duration

video_name = 'test_video.avi';
video_mode = "global";

% syms Ix Iy Iz m xdot_w ydot_w zdot_w
m = 0.65;
Ix = 0.0087408;
Iy = 0.0087408;
Iz = 0.0173188;
xdot_w = 0;
ydot_w = 0;
zdot_w = 0;

% constants = [Ix, Iy, Iz, Ax, Ay, Az, kdx, kdy, kdz, xdot_w, ydot_w, zdot_w, l, kf, km, ka, m, g];
constants = [Ix, Iy, Iz, 0.01, 0.01, 0.045, 0.1, 0.1, 0.1, xdot_w, ydot_w, zdot_w, 0.23, 3.13*(10^-5), 7.5*(10^-7), 1.0, m, 9.81]';
u_hover = sqrt(constants(17)*constants(18)/(4*constants(14)));

%% Obtain the quadrotor dynamics and the associated jacbian
QuadcopterModel;

%% Prepare the Non-Linear MPC
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
    Min={-1000;-1000;-1000;-1000}, ...
    Max={1000;1000;1000;1000}, ...
    RateMin={-1000;-1000;-1000;-1000}, ...
    RateMax={1000;1000;1000;1000} ...
    );

% Define the weights for the non-linear MPC
nlmpcobj.Weights.OutputVariables = [1 1 1 0 0 0 0 0 0 0 0 0]; % Define the output variables to be tracked
nlmpcobj.Weights.ManipulatedVariables = [MV_penalty MV_penalty MV_penalty MV_penalty];
nlmpcobj.Weights.ManipulatedVariablesRate = [MVrate_penallty MVrate_penallty MVrate_penallty MVrate_penallty];

%% Execute the simulation
% Specify the initial conditions
x = [0;0;5;0;0;0;0;0;0;0;0;0];

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
    yref = QuadrotorReferenceTrajectory(t);

    % Compute control move with reference previewing
    xk = xHistory(k,:);
    [uk,nloptions,info] = nlmpcmove(nlmpcobj,xk,lastMV,yref',[],nloptions);

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
T_series = 0:dt:Ttot;
xTarget = QuadrotorReferenceTrajectory(T_series)';

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


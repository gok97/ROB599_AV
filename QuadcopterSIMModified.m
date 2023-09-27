%% Mathematical Modeling and Simulation of Quadcopter Drone
%  Manuel Mariani, Daniele Manes; Ancona 2018

%% Prepare Workspace
clc
clear
close all

%% Variables & Parameters Declaration
% Parameters
Deltat = 0.01;                  % Value of discretization of the time interval (s)
m = 0.45;                       % Mass of drone (Kg)
l = 0.20;                       % Length of drone arms, from the center (m)
b = 7.5e-7 ;                    % Drag constant
L = 3e-6;                       % Lift constant
A = 0.25/m*eye(3);              % Aerodynamical effects matrix
cost = l*L;

Ixx = 5e-3;                     % Inertia about the X axis
Iyy = 5e-3;                     % Inertia about the Y axis
Izz = 8e-3;                     % Inertia about the Z axis
I = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];% Inertia matrix
Ir = 6e-5;                      % Rotor Inertia

% Initial Conditions
xi = zeros(3, 2);               % Relative position (x,y,z) of drone, relative to Earth Frame
xii = zeros(3,2);               % Derivate of xi
eta = zeros(3, 2);              % Attitudes of drone (roll, pitch, yaw)
tau = zeros(3,1);               % Torques of drone, relative to body frame attitudes
T = 0;                          % Thrust of drone on the z axis
vel = zeros(3, 2);              % Angular velocities relative to the Body Frame
Winv = zeros(3,3);              % Tranformation matrix (inverted)
R = zeros(3,1);                 % Rotation vector of drone's z axis
Rx = zeros(3,3,2);              % x Rotation matrix
Ry = zeros(3,3,2);              % y Rotation matrix
Rz = zeros(3,3,2);              % z Rotation matrix

w = zeros(4,1);                 % Angular Velocity Matrix(rad/s)
deltaW = 0.5;                   % Step size for controlling the motors

%% Plot preparation
k = 0;              % Counter

% Drone shape definition
Drone = l*[ 1 -1 0 0 0;
            0 0 0 1 -1;
            0 0 0 0 0];       
RotatedDrone = Drone;

% Main plot for 3D simulation
p = plot3(RotatedDrone(1,:,1),RotatedDrone(2,:,1),RotatedDrone(3,:,1),'k.-');
animLine = animatedline('MaximumNumPoints',1000,'Color','r');


% Set Figure Options
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
axisLim = 2;
axis([-axisLim axisLim -axisLim axisLim -axisLim axisLim])
grid on;
grid minor;

%% Simulation & Numeric implementation
while Simulate
    k = k+1;
    
    phi = eta(1,1);
    theta = eta(2,1);
    psi = eta(3,1);
    
    w1square = w(1)^2;
    w2square = w(2)^2;
    w3square = w(3)^2;
    w4square = w(4)^2;
    % Angular momentum
    tau(:) = [cost*(w4square-w2square);
              cost*(w3square-w1square);
              b*(w1square - w2square + w3square - w4square)];
    % Thrust on drone's z axis
    T = L*(w1square + w2square + w3square + w4square);
    % Angular velocities
    Gamma = Ir * cross(vel(:,1), [0;0;1]) * (w(1,1) - w(2,1) + w(3,1) - w(4,1));
    vel(:,1+1) = Deltat*( I\(-cross( vel(:,1), I*vel(:,1)) -Gamma +tau(:)) ) + vel(:,1);
    % Transformation matrix from Body to Earth frame
    Winv(:,:) = [ 1 sin(phi)*tan(theta) cos(phi)*tan(theta);
                  0 cos(phi) -sin(phi);
                  0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
    % Attitudes
    eta(:,1+1) = Deltat*( Winv(:,:)*vel(:,1)) + eta(:,1);
    %                   Rotation matrices
    % Defining 3rd column of rotation matrix
    R(:) = [cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);
            sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
            cos(theta)*cos(phi)];
    % Defining the three rotation matrices for 3d plotting
    Rx(:,:,1) = [ 1 0 0;
                  0 cos(phi) sin(phi);
                  0 -sin(phi) cos(phi)];
    Ry(:,:,1) = [cos(theta) 0 -sin(theta);
                 0 1 0;
                 sin(theta) 0 cos(theta)];
    Rz(:,:,1) = [cos(psi) sin(psi) 0;
                 -sin(psi) cos(psi) 0;
                 0 0 1];
  
    % Positions (absolute)
    xii(:,1+1) = Deltat*( -[0;0;10] + T/m * R(:) -A*xii(:,1)) + xii(:,1);
    xi(:,1+1) = (Deltat*xii(:,1)) + xi(:,1);  
    
    % Update the plot
    axis([-axisLim+xi(1,1) axisLim+xi(1,1) -axisLim+xi(2,1)...
          axisLim+xi(2,1) -axisLim+xi(3,1) axisLim+xi(3,1)]);
    
    
    % Rotation of drone
    RotatedDrone(:,:) = Rx(:,:,1)'*Ry(:,:,1)'*Rz(:,:,1)*(Drone) + xi(:,1);
    set(p, 'XData', RotatedDrone(1,:), ...
           'YData', RotatedDrone(2,:), ...
           'ZData', RotatedDrone(3,:));          
    % Drone's trail
    addpoints(animLine,xi(1,1),xi(2,1),xi(3,1));
    drawnow;
          
    %% Setting values of next iteration
    vel(:,1) = vel(:,2);
    eta(:,1) = eta(:,2);
    xii(:,1) = xii(:,2);
    xi(:,1) = xi(:,2);
end

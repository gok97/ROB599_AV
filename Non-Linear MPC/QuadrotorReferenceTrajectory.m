function [ xdesired ] = QuadrotorReferenceTrajectory( t )
% This function generates reference signal for nonlinear MPC controller
% used in the quadrotor path following example.

% Copyright 2019 The MathWorks, Inc.

%#codegen
x =6*sin(t/3);
y = -6*sin(t/3).*cos(t/3);
z = 6*cos(t/3);
xdot = zeros(1,length(t));
ydot = zeros(1,length(t));
zdot = zeros(1,length(t));
phi = zeros(1,length(t));
theta = zeros(1,length(t));
psi = zeros(1,length(t));
phidot = zeros(1,length(t));
thetadot = zeros(1,length(t));
psidot = zeros(1,length(t));

xdesired = [x;y;z;xdot;ydot;zdot;phi;theta;psi;phidot;thetadot;psidot];
end


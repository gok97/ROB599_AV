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

for i = 1:(length(t)-1)
    xdot(i) = (x(i+1)-x(i))/(t(i+1)-t(i));
    ydot(i) = (y(i+1)-y(i))/(t(i+1)-t(i));
    zdot(i) = (z(i+1)-z(i))/(t(i+1)-t(i));
end

xdot(length(t)) = x(length(t)-1);
ydot(length(t)) = y(length(t)-1);
zdot(length(t)) = z(length(t)-1);

phi = zeros(1,length(t));
theta = zeros(1,length(t));
psi = zeros(1,length(t));
phidot = zeros(1,length(t));
thetadot = zeros(1,length(t));
psidot = zeros(1,length(t));

xdesired = [x;y;z;xdot;ydot;zdot;phi;theta;psi;phidot;thetadot;psidot];
end


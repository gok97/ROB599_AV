pos = [0, 0, 0;
       3, 3, 3;
       5, 5, 5];
vel = [0, 0, 0;
       1, 0, 0;
       0, 0, 0];
pos = [0, 0, 0;
            0, 0, 30;
            10, 10, 30;
            90, 90, 30;
            100, 100, 30;
            100, 100, 0.5];

vel = [0, 0, 0;
            0, 0, 0;
            20, 20, 0;
            20, 20, 0;
            0, 0, 0;
            0, 0, 0];
% trajectory = waypointTrajectory(pos, Velocities=vel);
trajectory = multirotorFlightTrajectory(pos, vel, zeros(6, 3), zeros(6, 3), zeros(6, 3), zeros(6, 1), 1:6);

positions = [];
velocities = [];
accelerations = [];
i = 1;
while ~isDone(trajectory)
    [position,orientation,velocity,acceleration,angularVelocity] = trajectory();
    disp(i)
    i = i + 1;
    positions = [positions; position];
    velocities = [velocities; velocity];
    accelerations = [accelerations; acceleration];
end

plot(positions)
figure()
plot(velocities)
figure()
plot(accelerations)

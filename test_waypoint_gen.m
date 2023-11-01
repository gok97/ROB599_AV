pos = [0, 0, 0;
       3, 3, 3;
       5, 5, 5];
vel = [0, 0, 0;
       1, 0, 0;
       0, 0, 0];
trajectory = waypointTrajectory(pos, Velocities=vel);

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

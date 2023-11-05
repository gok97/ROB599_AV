clear
clc

% Define the center and radius of the sphere
x_center = 2;   % X-coordinate of the center
y_center = 2;   % Y-coordinate of the center
z_center = 2;   % Z-coordinate of the center
radius = 1;     % Radius of the sphere

% Create a grid of points for the sphere
[x, y, z] = sphere(500); % Adjust the resolution as needed

% Scale the sphere and translate it to the desired location
x = x * radius + x_center;
y = y * radius + y_center;
z = z * radius + z_center;

% Create a 3D plot of the sphere with semitransparent red color
surf(x, y, z, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

% Set view and lighting options
lighting gouraud; % Gouraud shading for smooth lighting
camlight; % Add a light source


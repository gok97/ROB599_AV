%% Prepare the workspace
clear;
clc;

rng(100,"twister")

%% Set the map parameters
% Map Settings
width = 500;
length = 500;

% Tree Settings
n_tree = 5000;
tree_size = [1, 1];

tree_mean_h = 15;
tree_std_h = 5/3;

n_branch_min = 3;
n_branch_max = 10;


%% Define the empty map
omap3D =  occupancyMap3D;
mapWidth = width;
mapLength = length;

%% Populate the map with obstacles
counter_tree = 1;
while counter_tree <= n_tree

    % Determine the tree height
    tree_height = tree_mean_h + tree_std_h*randn();

    % Generate the tree trunk
    xPosition = randi([0 mapWidth-tree_size(1)],1);
    yPosition = randi([0 mapLength-tree_size(2)],1);
    [xObstacle,yObstacle,zObstacle] = meshgrid(xPosition:xPosition+tree_size(1), yPosition:yPosition+tree_size(2), 0:tree_height);
    xyzObstacles = [xObstacle(:) yObstacle(:) zObstacle(:)];
    setOccupancy(omap3D,xyzObstacles,1)
    
    n_branches = n_branch_min + (n_branch_max-n_branch_min)*rand();
    counter_branch = 1;

    while counter_branch <= n_branches
        % Choose the height at which the branch should generate
        branch_height = 3 + (tree_height-3)*rand();

        % Set the branch size
        branch_width = 1 + (tree_height/4 - 1)*rand();
        branch_lenght = 1 + (tree_height/4 - 1)*rand();

        % Depending on the side the branch should generate on, create the obstacle
        side_value = rand();
        if side_value < 0.25
            xPosition_branch_start = (xPosition+1-branch_width) + (branch_width-1)*rand();
            yPosition_branch_start = yPosition-branch_lenght;
            
        elseif side_value < 0.5
            xPosition_branch_start = xPosition+1;
            yPosition_branch_start = (yPosition+1-branch_lenght) + (branch_lenght-1)*rand();

        elseif side_value < 0.75
            xPosition_branch_start = (xPosition+1-branch_width) + (branch_width-1)*rand();
            yPosition_branch_start = yPosition+1;

        else
            xPosition_branch_start = xPosition-branch_width;
            yPosition_branch_start = (yPosition+1-branch_lenght) + (branch_lenght-1)*rand();

        end

        % Generate the branch
        [xObstacle,yObstacle,zObstacle] = meshgrid(xPosition_branch_start:xPosition_branch_start+branch_width, yPosition_branch_start:yPosition_branch_start+branch_lenght, branch_height:branch_height+1);
        xyzObstacles = [xObstacle(:) yObstacle(:) zObstacle(:)];
        setOccupancy(omap3D,xyzObstacles,1)

        counter_branch = counter_branch+1;

    end

    % Add a canopy
    canopy_side = 4 + (8-4)*rand();
    canopy_height = 1 + (4-1)*rand();

    xPosition_canopy_start = xPosition - floor(canopy_side/2);
    xPosition_canopy_end = xPosition + ceil(canopy_side/2);
    yPosition_canopy_start = yPosition - floor(canopy_side/2);
    yPosition_canopy_end = yPosition + ceil(canopy_side/2);

    % Generate the canopy
    [xObstacle,yObstacle,zObstacle] = meshgrid(xPosition_canopy_start:xPosition_canopy_end, yPosition_canopy_start:yPosition_canopy_end, tree_height:tree_height+canopy_height);
    xyzObstacles = [xObstacle(:) yObstacle(:) zObstacle(:)];
    setOccupancy(omap3D,xyzObstacles,1)

    counter_tree = counter_tree + 1;
end

%% Add the ground
[xGround,yGround,zGround] = meshgrid(0:mapWidth,0:mapLength,0);
xyzGround = [xGround(:) yGround(:) zGround(:)];
setOccupancy(omap3D,xyzGround,1)

%% Display the generated map
figure("Name","3D Occupancy Map")
show(omap3D)

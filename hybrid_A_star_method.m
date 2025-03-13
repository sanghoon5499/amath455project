function path = codegenPathPlanner(mapData,startPose,goalPose)
    %#codegen
    
    % Create a binary occupancy map from custom map data
    map = binaryOccupancyMap(mapData);

    % Create a state space object
    stateSpace = stateSpaceSE2;

    % Update state space bounds to be the same as map limits.
    stateSpace.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

    % Construct a state validator object using the statespace and map object
    validator = validatorOccupancyMap(stateSpace, Map=map);
    
    % Set the validation distance for the validator
    validator.ValidationDistance = 0.01;
    
    % Assign the state validator object to the plannerHybridAStar object
    planner = plannerHybridAStar(validator);
    
    % Compute a path for the given start and goal poses
    pathObj = plan(planner, startPose, goalPose);
    
    % Extract the path poses from the path object
    path = pathObj.States;
end

% Custom maze (25x25 grid with walls defined by 1s and free space by 0s)
mazeGrid = zeros(50, 50);  % Initialize as free space

% Define the walls (1 = occupied)
%mazeGrid(1,:) = 1;    % Top border
%mazeGrid(end,:) = 1;  % Bottom border
%mazeGrid(:,1) = 1;    % Left border
%mazeGrid(:,end) = 1;  % Right border

% Unsafe zones
mazeGrid(1:20, 11:20) = 1;
mazeGrid(31:40, 11:20) = 1;
mazeGrid(16:25, 31:40) = 1;
mazeGrid(31:40, 31:40) = 1;

% Use the maze grid to create an occupancy matrix
mapData = mazeGrid;  

% Start and goal poses
startPose = [3 3 pi/2];  % [x, y, theta]
goalPose = [45 45 pi/2];

% Visualization
figure;
show(binaryOccupancyMap(mapData));
hold on;

% Start state and Goal state
scatter(startPose(1), startPose(2), "g", "filled");
scatter(goalPose(1), goalPose(2), "r", "filled");

% Call the path planner with the custom maze and plot the path
path = codegenPathPlanner(mapData, startPose, goalPose);
plot(path(:,1), path(:,2), "r-", LineWidth=2);

%legend("Start Pose", "Goal Pose", "MATLAB Generated Path");
%legend("Location", "northwest");
grid on;
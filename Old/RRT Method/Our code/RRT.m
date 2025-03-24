

%%%%%%     1. Set up the environment     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 1a. Create occupancy map (rows, cols, resolution)
occGrid = occupancyMap(5, 5, 10);

% 1b. Initialize the occupancy map with empty space, else it's initialized
%       as void space (-1), which is unnavigatable.
%     - (start, end, #spaces between start and end)
[x, y] = meshgrid(linspace(0, 5, 51), linspace(0, 5, 51));
xy = [x(:), y(:)];
setOccupancy(occGrid, xy, 0);

% 1c. Set the obstacles
[x, y] = meshgrid(linspace(1.1, 2), linspace(3.1, 5));
xy = [x(:), y(:)];
setOccupancy(occGrid, xy, 1);
[x, y] = meshgrid(linspace(1.1, 2), linspace(1.1, 2));
xy = [x(:), y(:)];
setOccupancy(occGrid, xy, 1);
[x, y] = meshgrid(linspace(3.1, 4), linspace(1.1, 2));
xy = [x(:), y(:)];
setOccupancy(occGrid, xy, 1);
[x, y] = meshgrid(linspace(3.1, 4), linspace(2.6, 3.5));
xy = [x(:), y(:)];
setOccupancy(occGrid, xy, 1);

show(occGrid);

% 1d. Set start and goal poses.
start = [0.25, 0.25, pi/2];  % [x, y, theta]
goal = [4.5, 4.5, 0];

% 1e. Show start and goal positions of robot.
hold on
plot(start(1),start(2),'ro')
plot(goal(1),goal(2),'mo')
hold off


%%%%%%     2. Original map inflation     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Make a copy of the original map and infate it by 0.1 meters. 
%    - Use this inflated map for path planning. 
%    - Use the original map for visualization purpose. 
inflatedMap = copy(occGrid);
inflate(inflatedMap,0.1); 
show(inflatedMap);


%%%%%%     3. Set up state space     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    - currently using example code's dubin's method
%    - need to change this such that we use the A and B matrix from the
%        project outline.
bounds = [inflatedMap.XWorldLimits; inflatedMap.YWorldLimits; [-pi pi]];
ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.2;


%%%%%%     4. Validate and plan     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Validate the state and plan the path to target
stateValidator = validatorOccupancyMap(ss); 
stateValidator.Map = inflatedMap;
stateValidator.ValidationDistance = 0.05;

planner = plannerRRT(ss,stateValidator);
planner.MaxConnectionDistance = 0.25;
planner.MaxIterations = 30000;


%%%%%%     5. Check if target reached     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
planner.GoalReachedFcn = @exampleHelperCheckIfGoal;

function isReached = exampleHelperCheckIfGoal(planner, goalState, newState)
    isReached = false;
    threshold = 0.1;
    if planner.StateSpace.distance(newState, goalState) < threshold
        isReached = true;
    end
end

%%%%%%     5. Plan pt.2 and length calc     %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% "Reset the random number generator to ensure reproducible results. Plan
% the path from the start to the goal pose."
rng default
[pthObj,solnInfo] = plan(planner,start,goal);

% we don't actually need this info
shortenedPath = shortenpath(pthObj,stateValidator);
originalLength = pathLength(pthObj);
shortenedLength = pathLength(shortenedPath);


%%%%%%     6. Plot results     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
show(occGrid)
hold on

% Plot entire search tree.
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),plannerLineSpec.tree{:})

% Interpolate and plot path.
interpolate(pthObj,300)
plot(pthObj.States(:,1),pthObj.States(:,2),plannerLineSpec.path{:})

% Interpolate and plot path.
interpolate(shortenedPath,300);
plot(shortenedPath.States(:,1),shortenedPath.States(:,2),'g-','LineWidth',3)

% Show start and goal in grid map.
plot(start(1),start(2),'ro')
plot(goal(1),goal(2),'mo')
legend('search tree','original path','shortened path', 'Location', 'northwest')
hold off














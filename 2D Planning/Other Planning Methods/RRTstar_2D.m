% Define original superellipse parameters
clc;clear;close all;
tic
superellipses_original = [
    0.0673554782636699	0.0689576082024499	1.01007586094759	0.296194685907200	-0.174536214946745	-2.79222308821738;
    0.0340295535849556	0.0659625880289237	0.0945537135123447	0.381786466689170	-0.0722233215126290	-2.22233009241378;
    0.0521446041515254	0.0351803824503460	0.159659543191639	0.137036912664411	-0.198639544958104	1.62650781418238;
    0.0357651835637329	0.0349964770985397	0.226706897873248	0.135841649546474	-0.135097286683826	1.53352141203512;
    0.0342646170996485	0.0345984413894670	0.111619106118759	0.234588947338938	-0.0650102329067118	3.15724743228591;
    0.0343028914441385	0.0341800477259898	0.0869580898838730	0.234288502557136	-0.0179071551079991	1.52677014555401;
    0.0345298888513869	0.0349615120388748	0.159838157548769	0.135003590103764	-0.0349963425929118	-1.86213602247859;
    0.0358937942069177	0.0348269957041967	0.192622812265083	0.394895966251423	-0.0221823306398295	-2.73692008148004;
];

start_point = [0.0489, -0.1493];
end_point = [0.4,-0.17];

% Find minimum x and y coordinates
min_x = min(superellipses_original(:, 4));
min_x = min([min_x,start_point(1),end_point(1)]);
min_y = min(superellipses_original(:, 5));
min_y = min([min_y,start_point(2),end_point(2)]);

% Shift to ensure all are in the first quadrant
shift_x = abs(min_x);
shift_y = abs(min_y) + 0.1;

superellipses = superellipses_original;
superellipses(:, 4) = superellipses(:, 4) + shift_x;
superellipses(:, 5) = superellipses(:, 5) + shift_y;

start_point = start_point + [shift_x,shift_y];
end_point = end_point +  [shift_x,shift_y];

% Enlarge the superellipse parameters for visualization
superellipses_enlarged = superellipses;
superellipses_enlarged(:, 1) = 1.1 * superellipses(:, 1); % Enlarge 'a'
superellipses_enlarged(:, 2) = 1.1 * superellipses(:, 2); % Enlarge 'b'

% Create the state space
ss = stateSpaceSE2;

% Create an occupancy map
map = binaryOccupancyMap(0.5, 0.5, 500); % Increased resolution to 500 cells/meter

% Mark the enlarged superellipse obstacles on the occupancy map
for i = 1:size(superellipses_enlarged, 1)
    % Generate points along the boundary of the enlarged superellipse
    [x_trans, y_trans] = sampleUniformSuperellipse(superellipses_enlarged(i, 1), ...
        superellipses_enlarged(i, 2), superellipses_enlarged(i, 3), superellipses_enlarged(i, 6), ...
        superellipses_enlarged(i, 4), superellipses_enlarged(i, 5), 2000);
    
    % Set occupancy of boundary points with a small buffer
    buffer_size = 0.002; % Increased buffer size
    for j = 1:length(x_trans)
        setOccupancy(map, [x_trans(j) y_trans(j)], 1);
        setOccupancy(map, [x_trans(j) y_trans(j)] + buffer_size * [1, 0], 1);
        setOccupancy(map, [x_trans(j) y_trans(j)] - buffer_size * [1, 0], 1);
        setOccupancy(map, [x_trans(j) y_trans(j)] + buffer_size * [0, 1], 1);
        setOccupancy(map, [x_trans(j) y_trans(j)] - buffer_size * [0, 1], 1);
    end
end

% Create an occupancyMap-based state validator using the created state space
sv = validatorOccupancyMap(ss);
sv.Map = map;
sv.ValidationDistance = 0.001; % Decreased validation distance

% Update state space bounds to be the same as map limits
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

% Create RRT* path planner with adjusted parameters
planner = plannerRRTStar(ss, sv);
planner.MaxConnectionDistance = 0.012; % Increased from 0.005
planner.MaxIterations = 20000; % Increased from 10000
planner.GoalBias = 0.1; % Added goal bias
planner.GoalReachedFcn = @(planner, goal, state) norm(state(1:2) - goal(1:2)) < 0.005; % Adjusted goal reached function

% Set start and goal states
start_point(3) = 0;
end_point(3) =  0;

% Validate the start and end points using the state validator
isStartValid = isStateValid(sv, start_point);
isEndValid = isStateValid(sv, end_point);

if ~isStartValid
    error('Start point is in collision or out of bounds.');
end

if ~isEndValid
    error('End point is in collision or out of bounds.');
end

% Plan a path with updated settings
rng(100,'twister') % repeatable result
[pthObj, solnInfo] = plan(planner, start_point, end_point);

% Visualize the results
figure;
hold on;

% Plot tree expansion
%plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.-', 'Color', [0.7 0.7 0.7]);

% Draw path
if ~isempty(pthObj.States)
    plot(pthObj.States(:,1), pthObj.States(:,2), 'r-', 'LineWidth', 2);
else
    disp('No valid path found');
end

% Plot and fill the original superellipses
for i = 1:size(superellipses, 1)
    [x_trans, y_trans] = sampleUniformSuperellipse(superellipses(i, 1), ...
        superellipses(i, 2), superellipses(i, 3), superellipses(i, 6), ...
        superellipses(i, 4), superellipses(i, 5), 100);
    fill(x_trans, y_trans, 'b', 'FaceAlpha', 0.3);
end

% Mark start and end points
plot(start_point(1), start_point(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(end_point(1), end_point(2), 'mo', 'MarkerSize', 10, 'LineWidth', 2);

xlabel('X');
ylabel('Y');
axis equal;
axis([0.05 0.55 0 0.4])
%axis off;
hold off;

pathCoords = [pthObj.States(:,1), pthObj.States(:,2)];
path_len = path_length(pathCoords);

  % Calculate the minimum distance from path to any superellipse
min_distances = zeros(size(pathCoords, 1), 1); % To store min distance for each waypoint

for j = 1:size(pathCoords, 1)
    waypoint = pathCoords(j, :); % Get current waypoint
    waypoint_min_dist = Inf; % Initialize minimum distance for this waypoint
    
    % Loop through all superellipses
    for i = 1:size(superellipses, 1)
        a = superellipses(i, 1);
        b = superellipses(i, 2);
        n = superellipses(i, 3);
        h = superellipses(i, 4);
        k = superellipses(i, 5);
        theta = superellipses(i, 6);
        
        % Get uniformly spaced points on the superellipse
        [x_trans, y_trans] = sampleUniformSuperellipse(a, b, n, theta, h, k, 100);
        superellipse_points = [x_trans', y_trans'];
        
        % Calculate distances from the waypoint to all points on this superellipse
        distances = pdist2(waypoint, superellipse_points);
        
        % Find the minimum distance to this superellipse
        min_dist_to_superellipse = min(distances);
        
        % Update the minimum distance for this waypoint
        waypoint_min_dist = min(waypoint_min_dist, min_dist_to_superellipse);
    end
    
    % Store the minimum distance for this waypoint
    min_distances(j) = waypoint_min_dist;
end

% Find the minimum distance among all waypoints
min_dist = min(min_distances);
fprintf('Total path length: %.4f\n', path_len);
fprintf('Minimum distance from path to obstacles: %.4f\n', min_dist);
fprintf("Time taken : %.4f\n", toc)
% Superellipse sampling function
function [x, y] = sampleUniformSuperellipse(a, b, n, theta, h, k, numPoints)
    % Generate initial raw points
    t_raw = linspace(0, 2*pi, numPoints);
    [x_raw, y_raw] = superellipse(t_raw, a, b, n);
    % Calculate the cumulative arc length
    x_diff = diff(x_raw);
    y_diff = diff(y_raw);
    arc_length = [0, cumsum(sqrt(x_diff.^2 + y_diff.^2))];
    % Interpolate to get uniform arc length
    uniform_arc_length = linspace(0, arc_length(end), numPoints);
    x_interp = interp1(arc_length, x_raw, uniform_arc_length, 'linear');
    y_interp = interp1(arc_length, y_raw, uniform_arc_length, 'linear');
    [x, y] = transform(x_interp, y_interp, theta, h, k);
end

% Superellipse parameterization function
function [x, y] = superellipse(t, a, b, n)
    x = a * sign(cos(t)) .* abs(cos(t)).^n;
    y = b * sign(sin(t)) .* abs(sin(t)).^n;
end

% Transformation function
function [x, y] = transform(x, y, theta, h, k)
    x_rot = x * cos(theta) - y * sin(theta);
    y_rot = x * sin(theta) + y * cos(theta);
    x = x_rot + h;
    y = y_rot + k;
end

function path_len = path_length(path_coords)
    path_len = 0;
    n = size(path_coords,1);
    for i = 1:n-1
        path_len = path_len + norm(path_coords(i+1,:)-path_coords(i,:));
    end
end



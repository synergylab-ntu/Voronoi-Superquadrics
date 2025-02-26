clc; clear; close all;
% Define original superellipse parameters
% superellipses_original = [
%         2, 1, 0.3, 5, 2, pi/3;
%         1.5, 2, 0.5, -1, -1, pi/6;
%         4, 3, 0.8, 7, -10, -pi/6;
%         0.5, 3, 1.2, 11, 0, pi/12;
%         3, 2, 0.6, 0, 8, -0.7*pi;
%         2, 3, 0.1, 15, 20, -0.2*pi;
%         1, 5, 1, -15, -10, 0.3*pi;
%         1.5, 1, 1.3, -4, -6, 0.5*pi;
%         1.8, 4, 1.5, -10, 15, 0.6*pi;
%         3, 4, 1.8, 20, 6, 0.8*pi;
%         4, 2, 1.6, 19, -7, 0.2*pi;
%     ];

tic
superellipses = [
        0.0673554782636699	0.0689576082024499	1.01007586094759	0.296194685907200	-0.174536214946745	-2.79222308821738;
        0.0340295535849556	0.0659625880289237	0.0945537135123447	0.381786466689170	-0.0722233215126290	-2.22233009241378;
        0.0521446041515254	0.0351803824503460	0.159659543191639	0.137036912664411	-0.198639544958104	1.62650781418238;
        0.0357651835637329	0.0349964770985397	0.226706897873248	0.135841649546474	-0.135097286683826	1.53352141203512;
        0.0342646170996485	0.0345984413894670	0.111619106118759	0.234588947338938	-0.0650102329067118	3.15724743228591;
        0.0343028914441385	0.0341800477259898	0.0869580898838730	0.234288502557136	-0.0179071551079991	1.52677014555401;
        0.0345298888513869	0.0349615120388748	0.159838157548769	0.135003590103764	-0.0349963425929118	-1.86213602247859;
        0.0358937942069177	0.0348269957041967	0.192622812265083	0.394895966251423	-0.0221823306398295	-2.73692008148004;
    ];

% % Define start and end points
% start_point = [-24, -10];
% end_point = [20, 10];

% Define start and end points
start_point = [0.0489, -0.1493];
end_point = [0.4,-0.17];

% Find minimum x and y coordinates
min_x = min(superellipses(:, 4));
min_x = min([min_x,start_point(1),end_point(1)]);
min_y = min(superellipses(:, 5));
min_y = min([min_y,start_point(2),end_point(2)]);

% Shift to ensure all are in the first quadrant
shift_x = abs(min_x);
shift_y = abs(min_y)+0.1;

superellipses(:, 4) = superellipses(:, 4) + shift_x;
superellipses(:, 5) = superellipses(:, 5) + shift_y;

start_point = start_point + [shift_x,shift_y];
end_point = end_point +  [shift_x,shift_y];
% Enlarge the superellipse parameters for visualization
superellipses_enlarged = superellipses;
superellipses_enlarged(:, 1) = 1 * superellipses(:, 1); % Enlarge 'a'
superellipses_enlarged(:, 2) = 1 * superellipses(:, 2); % Enlarge 'b'

% Define grid resolution and map size
grid_resolution = 0.005; % Smaller value for higher resolution
map_size = [0.5, 0.5]; % Size of the map

% Create a fine grid for the map
[x, y] = meshgrid(0:grid_resolution:map_size(1), 0:grid_resolution:map_size(2));
points = [x(:), y(:)];

% Check if each point is inside any superellipse
inside = false(size(points, 1), 1);
for i = 1:size(superellipses_enlarged, 1)
    a = superellipses_enlarged(i, 1);
    b = superellipses_enlarged(i, 2);
    n = superellipses_enlarged(i, 3);
    h = superellipses_enlarged(i, 4);
    k = superellipses_enlarged(i, 5);
    theta = superellipses_enlarged(i, 6);

    % Rotate points back to align with superellipse
    rot_matrix = [cos(theta), sin(theta); -sin(theta), cos(theta)];
    rotated_points = (rot_matrix * (points' - [h; k]))';

    % Check if points are inside the superellipse
    inside = inside | ((abs(rotated_points(:,1) / a).^(2/n) + abs(rotated_points(:,2) / b).^(2/n)) <= 1);
end

% Filter out points that are inside superellipses
points = points(~inside, :);

% Create a graph
G = graph();

% Add nodes to the graph
G = addnode(G, size(points, 1));

% Calculate distances between points
D = pdist2(points, points);

% Define neighbors within the 8 direction neighborhood
neighbors = (D <= 1.4*sqrt(2) * grid_resolution) & (D > 0);

% Add edges to the graph with weights
[rows, cols] = find(neighbors);
weights = D(neighbors);
G = addedge(G, rows, cols, weights);



% Find the nearest nodes to the start and end points
[~, start_idx] = min(pdist2(points, start_point));
[~, end_idx] = min(pdist2(points, end_point));

% Find the shortest path using Dijkstra's algorithm
[path_idx, path_dist] = shortestpath(G, start_idx, end_idx);

% Get the coordinates of the path
path = points(path_idx, :);

% Initialize total path length
path_length = 0;

% Calculate distance from start point to first node in the path
start_to_first_node_dist = pdist2(start_point, path(1, :));
path_length = path_length + start_to_first_node_dist;

% Calculate the distances between consecutive points along the path
for i = 1:size(path, 1) - 1
    segment_length = pdist2(path(i, :), path(i + 1, :));
    path_length = path_length + segment_length;
end

% Calculate distance from last node in the path to the end point
last_node_to_end_dist = pdist2(path(end, :), end_point);
path_length = path_length + last_node_to_end_dist;

% Display the total path length

% Calculate the minimum distance from path to any superellipse
min_distances = zeros(size(path, 1), 1); % To store min distance for each waypoint

for j = 1:size(path, 1)
    waypoint = path(j, :); % Get current waypoint
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
min_distance_to_obstacles = min(min_distances);

% Display the minimum distance between the path and obstacles
fprintf('Total path length: %.4f\n', path_length);
fprintf('Minimum distance from path to obstacles: %.4f\n', min_distance_to_obstacles);
fprintf("Time taken : %.4f\n", toc)

% Visualize the results
figure;
hold on;

% % Plot the map grid
% plot(points(:,1), points(:,2), 'b.', 'MarkerSize', 5);

% Plot the path
plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2);

% Plot and fill the original superellipses
for i = 1:size(superellipses, 1)
    [x_trans, y_trans] = sampleUniformSuperellipse(superellipses(i, 1), ...
        superellipses(i, 2), superellipses(i, 3), superellipses(i, 6), ...
        superellipses(i, 4), superellipses(i, 5), 100);
    fill(x_trans, y_trans, 'b', 'FaceAlpha', 0.3);
end
% Params_test = matrix_cell(superellipses);
% hold on
% plot_multiple_superellipses(Params_test)

% Mark start and end points
plot(start_point(1), start_point(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(end_point(1), end_point(2), 'mo', 'MarkerSize', 10, 'LineWidth', 2);

title('Graph-based Path Planning with Superellipse Obstacles');
xlabel('X');
ylabel('Y');
axis equal;
axis([0.05 0.55 0 0.4])
hold off;

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




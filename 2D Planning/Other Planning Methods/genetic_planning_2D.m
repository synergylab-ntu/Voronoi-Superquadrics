% Constants
clc;clear;close all; 
tic
POPULATION_SIZE = 100;
NUM_GENERATIONS = 5; % Increased from 30 to 100
MUTATION_RATE = 0.1; % Changed from 0 to 0.1
MAX_WAYPOINTS = 150;
NUM_OBSTACLES = 8;  % Number of superellipses
SPACE_SIZE = 100;
SMOOTHNESS_WEIGHT = 0.1;
STEP_SIZE = 0.03;  % Uniform distance between points
DIRECTION_BIAS = 0.5;  % Bias towards the goal direction
PADDING = 0;
GOAL_THRESHOLD = STEP_SIZE ;  % Increased threshold to consider goal reached

% Define parameters for the superellipses (a, b, n, center_x, center_y, theta)
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

% Define start and goal points
start = [0.0489, -0.1493];
goal = [0.4,-0.17];

% Run the genetic algorithm
best_path = genetic_algorithm(POPULATION_SIZE, NUM_GENERATIONS, start, goal, MAX_WAYPOINTS, STEP_SIZE, SPACE_SIZE, GOAL_THRESHOLD, DIRECTION_BIAS, MUTATION_RATE, SMOOTHNESS_WEIGHT, superellipses);
pathCoords = best_path;
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

% Plot the best path
plot_path(best_path, start, goal, superellipses);
hold on;
% Plot start and end points
scatter(start(1), start(2), 'g', 'filled');
scatter(goal(1), goal(2), 'magenta', 'filled');
axis equal;
axis([0 0.48 -0.28 0.05])
fprintf('\nTotal path length: %.4f\n', path_len);
fprintf('Minimum distance from path to obstacles: %.4f\n', min_dist);
fprintf("Time taken : %.4f\n", toc)

function direction = biased_random_direction(prev_point, goal, DIRECTION_BIAS)
    goal_direction = goal - prev_point;
    goal_direction = goal_direction / norm(goal_direction);
    
    random_direction = randn(1, 2);
    random_direction = random_direction / norm(random_direction);
    
    biased_direction = DIRECTION_BIAS * goal_direction + (1 - DIRECTION_BIAS) * random_direction;
    direction = biased_direction / norm(biased_direction);
end

function path = create_individual(start, goal, MAX_WAYPOINTS, STEP_SIZE, SPACE_SIZE, GOAL_THRESHOLD, DIRECTION_BIAS, superellipses)
    path = start;
    for i = 1:MAX_WAYPOINTS
        direction = biased_random_direction(path(end, :), goal, DIRECTION_BIAS);
        new_point = path(end, :) + STEP_SIZE * direction;
        
        % Check for collision and regenerate if necessary
        attempts = 0;
        while any(arrayfun(@(j) is_point_in_superellipse(new_point, superellipses(j,:)), 1:size(superellipses,1))) && attempts < 10
            direction = biased_random_direction(path(end, :), goal, DIRECTION_BIAS);
            new_point = path(end, :) + STEP_SIZE * direction;
            attempts = attempts + 1;
        end
        
        path = [path; new_point];
        if norm(new_point - goal) < GOAL_THRESHOLD
            break;
        end
    end
end

function population = initialize_population(POPULATION_SIZE, start, goal, MAX_WAYPOINTS, STEP_SIZE, SPACE_SIZE, GOAL_THRESHOLD, DIRECTION_BIAS, superellipses)
    population = cell(1, POPULATION_SIZE);
    for i = 1:POPULATION_SIZE
        population{i} = create_individual(start, goal, MAX_WAYPOINTS, STEP_SIZE, SPACE_SIZE, GOAL_THRESHOLD, DIRECTION_BIAS, superellipses);
    end
end

function fit = fitness(individual, start, goal, GOAL_THRESHOLD, SPACE_SIZE, SMOOTHNESS_WEIGHT, superellipses)
    path = [start; individual; goal];
    
    % Calculate total distance
    total_distance = sum(sqrt(sum(diff(path).^2, 2)));
    
    % Calculate smoothness penalty
    angles = diff(atan2(diff(path(:,2)), diff(path(:,1))));
    smoothness_penalty = sum(abs(angles));
    
    % Check for intersections with superellipses
    collision_penalty = 0;
    for i = 1:size(path, 1)-1
        p1 = path(i, :);
        p2 = path(i+1, :);
        for j = 1:size(superellipses, 1)
            if line_superellipse_intersection(p1, p2, superellipses(j, :))
                collision_penalty = collision_penalty + 1000;
            end
        end
    end
    
    % Check if the path reaches the goal
    goal_reached = norm(path(end, :) - goal) < GOAL_THRESHOLD;
    if goal_reached
        goal_penalty = 0;
    else
        goal_penalty = 10000;
    end
    
    % Combine all factors
    total_penalty = total_distance + SMOOTHNESS_WEIGHT * smoothness_penalty + collision_penalty + goal_penalty;
    
    fit = 1 / (total_penalty + 1e-6);  % Add small constant to avoid division by zero
end

function parents = select_parents(population, fitnesses)
    [~, sorted_indices] = sort(fitnesses, 'descend');
    top_indices = sorted_indices(1:ceil(length(population)/2));
    idx1 = top_indices(randi(length(top_indices)));
    idx2 = top_indices(randi(length(top_indices)));
    while idx2 == idx1
        idx2 = top_indices(randi(length(top_indices)));
    end
    parents = {population{idx1}, population{idx2}};
end

function child = crossover(parent1, parent2)
    min_length = min(size(parent1, 1), size(parent2, 1));
    crossover_point = randi(min_length);
    child = [parent1(1:crossover_point, :); parent2(crossover_point+1:end, :)];
end

function mutated = mutate(individual, MUTATION_RATE, STEP_SIZE)
    mutated = individual;
    for i = 1:size(individual, 1)
        if rand < MUTATION_RATE
            mutated(i, :) = mutated(i, :) + randn(1, 2) * STEP_SIZE;
        end
    end
end

function best_individual = genetic_algorithm(POPULATION_SIZE, NUM_GENERATIONS, start, goal, MAX_WAYPOINTS, STEP_SIZE, SPACE_SIZE, GOAL_THRESHOLD, DIRECTION_BIAS, MUTATION_RATE, SMOOTHNESS_WEIGHT, superellipses)
    population = initialize_population(POPULATION_SIZE, start, goal, MAX_WAYPOINTS, STEP_SIZE, SPACE_SIZE, GOAL_THRESHOLD, DIRECTION_BIAS, superellipses);
    best_fitness = -Inf;
    best_individual = [];
    
    for generation = 1:NUM_GENERATIONS
        fitnesses = zeros(1, POPULATION_SIZE);
        for i = 1:POPULATION_SIZE
            fitnesses(i) = fitness(population{i}, start, goal, GOAL_THRESHOLD, SPACE_SIZE, SMOOTHNESS_WEIGHT, superellipses);
        end
        
        [current_best_fitness, best_idx] = max(fitnesses);
        current_best_individual = population{best_idx};
        
        if current_best_fitness > best_fitness
            best_fitness = current_best_fitness;
            best_individual = current_best_individual;
        end
        
        fprintf('Generation %d: Best fitness = %f, Path length = %d\n', ...
            generation, best_fitness, size(best_individual, 1));
        
        new_population = cell(1, POPULATION_SIZE);
        for i = 1:2:POPULATION_SIZE
            parents = select_parents(population, fitnesses);
            child1 = crossover(parents{1}, parents{2});
            child2 = crossover(parents{2}, parents{1});
            child1 = mutate(child1, MUTATION_RATE, STEP_SIZE);
            child2 = mutate(child2, MUTATION_RATE, STEP_SIZE);
            new_population{i} = child1;
            new_population{i+1} = child2;
        end
        
        population = new_population;
    end
end

function intersects = line_superellipse_intersection(p1, p2, superellipse_params)
    num_samples = 100;
    t = linspace(0, 1, num_samples);
    intersects = false;
    for i = 1:num_samples
        point = p1 + t(i) * (p2 - p1);
        if is_point_in_superellipse(point, superellipse_params)
            intersects = true;
            return;
        end
    end
end

function inside = is_point_in_superellipse(point, superellipse_params)
    a = superellipse_params(1);
    b = superellipse_params(2);
    n = superellipse_params(3);
    cx = superellipse_params(4);
    cy = superellipse_params(5);
    theta = superellipse_params(6);
    
    x = point(1);
    y = point(2);
    
    x_rot = (x - cx) * cos(theta) + (y - cy) * sin(theta);
    y_rot = -(x - cx) * sin(theta) + (y - cy) * cos(theta);
    
    left_side = (abs(x_rot) .^ (2 / n)) / (a ^ (2 / n)) + (abs(y_rot) .^ (2 / n)) / (b ^ (2 / n));
    inside = left_side <= 1;
end

function plot_path(path, start, goal, superellipses)
    figure;
    hold on;
    
    % Plot superellipses
    t = linspace(0, 2*pi, 400);
    for i = 1:size(superellipses, 1)
        a = superellipses(i, 1);
        b = superellipses(i, 2);
        n = superellipses(i, 3);
        cx = superellipses(i, 4);
        cy = superellipses(i, 5);
        theta = superellipses(i, 6);
        [x, y] = superellipse(t, a, b, n);
        [x, y] = transform(x, y, theta, cx, cy);
        fill(x, y, 'b', 'FaceAlpha', 0.3);
    end
    
    % Plot path
    path_array = [start; path; goal];
    plot(path_array(:, 1), path_array(:, 2), 'r-', 'LineWidth', 2);
    axis equal;
    hold off;
end

function [x, y] = superellipse(t, a, b, n)
    x = a * sign(cos(t)) .* abs(cos(t)).^(n);
    y = b * sign(sin(t)) .* abs(sin(t)).^(n);
end

function [x_trans, y_trans] = transform(x, y, theta, h, k)
    x_rot = x * cos(theta) - y * sin(theta);
    y_rot = x * sin(theta) + y * cos(theta);
    x_trans = x_rot + h;
    y_trans = y_rot + k;
end

function path_len = path_length(path_coords)
    path_len = 0;
    n = size(path_coords,1);
    for i = 1:n-1
        path_len = path_len + norm(path_coords(i+1,:)-path_coords(i,:));
    end
end

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
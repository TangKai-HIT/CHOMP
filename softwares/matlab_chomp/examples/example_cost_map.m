%% 
% Copyright (c) 2016 Carnegie Mellon University, Sanjiban Choudhury <sanjibac@andrew.cmu.edu>
%
% For License information please see the LICENSE file in the root directory.
%
%% EXAMPLE_COST_MAP
% Create sample cost map and then do unconstrained chomp
clc;
clear;
close all;

%% Create cost map
bbox = [0 1 0 1]; %unit bounding box
% rectangle 1
rectangle_array(1).low = [0.2 0.2];
rectangle_array(1).high = [0.4 0.5];
% rectangle 2
rectangle_array(2).low = [0.6 0.5];
rectangle_array(2).high = [0.8 0.9];
resolution = 0.001; %resolution of map

% Get map
map = rectangle_maps( bbox, rectangle_array, resolution);

% Cost map 
epsilon = 0.15; % horizon of expansion
cost_map = create_costmap_sqdist(map, epsilon);

% Cost map derivatives
[cost_map_x, cost_map_y] = get_cost_map_derivatives(cost_map);

% Visualize cost map
figure;
visualize_cost_map(cost_map);

%% Create a trajectory
p_start = [0 0.5];
p_end = [1 0.5];
N = 100;
xi = get_wpset(p_start, p_end, N);
% xi = get_user_input_trajectory( p_start, p_end, N ); % (Get user traj)

%% Create cost function
lambda = 70;

[ A, b, c ] = create_smooth_matrices( p_start, p_end, size(xi,1) - 2 );
c_final = @(xi) costfn_smooth_value_wpset(xi, A, b, c) + lambda*value_arclength_wpset_map( xi, cost_map, p_start );
grad_final = @(xi) costfn_smooth_grad_wpset(xi, A, b) + lambda*gradient_arclength_wpset_map( xi, cost_map, cost_map_x, cost_map_y, p_start);

%% Create constraints
constraints = get_empty_constraints();

%% Chomp options
options = get_chomp_options();
options.Eta = 4200;
options.MinIter = 1;
options.MaxIter = 100;
options.TolFun = 1e-3;
options.Metric = A;
% options.Metric = eye(size(xi,1)-2);
options.MetricInverse = inv(options.Metric);
options.DecreaseStepSize = 0;

%% Perform chomp
tic;
[ xi_best, cost_best, exitflag, output ] = covariant_gradient_descent( xi, c_final, grad_final, constraints, options );
solved_time = toc;

%% Visualize
plot_chomp_history( output );
figure, plot_cost_history( output );

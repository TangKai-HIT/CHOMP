%% 
% Copyright (c) 2016 Carnegie Mellon University, Sanjiban Choudhury <sanjibac@andrew.cmu.edu>
%
% For License information please see the LICENSE file in the root directory.
%
%% EXAMPLE_COST_MAP_CONSTRAINED_GOAL_EQUALITY
% Create sample cost map and then do chomp where goal is equality
% constrained
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
hold on;

%% Create a trajectory
p_start = [0 0.5];
p_end = [1 0.5];
N = 100;
xi = get_wpset(p_start, p_end, N);
% xi = get_user_input_trajectory( p_start, p_end, N ); % (Get user traj)
%% Create cost function
lambda = 100;

[ A, b, c ] = create_smooth_matrices_freegoal( p_start, size(xi,1) - 2 );
c_final = @(xi) costfn_smooth_value_wpset(xi, A, b, c) + lambda*value_arclength_wpset_map( xi, cost_map, p_start );
grad_final = @(xi) costfn_smooth_grad_wpset(xi, A, b) + lambda*gradient_arclength_wpset_map( xi, cost_map, cost_map_x, cost_map_y, p_start);

%% Create a goal equality constraint on X
pt = {0.95, []};
constraints = get_goal_equality(pt, size(xi,1) - 2 ); 
hold on;
plot([pt{1} pt{1}], [-1e6 1e6], 'r--', 'LineWidth', 3);

%% Chomp options
options = get_chomp_options();
options.Eta = 5000;
options.MinIter = 1;
options.MaxIter = 100;
options.TolFun = 1e-3;
options.MetricInverse = inv(A);
options.Metric = A;
options.DecreaseStepSize = 1;
options.FreeEndPoint = 1;

%% Perform chomp
[ xi_best, cost_best, exitflag, output ] = covariant_gradient_descent( xi, c_final, grad_final, constraints, options );


%% Visualize
plot_chomp_history( output );
figure, plot_cost_history( output );

%% 
% Copyright (c) 2016 Carnegie Mellon University, Sanjiban Choudhury <sanjibac@andrew.cmu.edu>
%
% For License information please see the LICENSE file in the root directory.
%
%% EXAMPLE_SHAPE
% Create shape array and perform chomp
clc;
clear;
close all;

%% Create cost map
bbox = [0 1 0 1]; %unit bounding box
% rectangle 1
rectangle_array(1) = get_rectangle_shape(0.2,0.2,0.2,0.3);
% rectangle 2
rectangle_array(2) = get_rectangle_shape(0.6,0.5,0.2,0.4);

% Visualize 
figure;
visualize_shapes(rectangle_array);
axis(bbox);

%% Create a trajectory
p_start = [0 0.5];
p_end = [1 0.5];
N = 100;
xi = get_wpset(p_start, p_end, N);

%% Create cost function
lambda = 100;

[ A, b, c ] = create_smooth_matrices( p_start, p_end, size(xi,1) - 2 );

epsilon = 0.15;
c_obs_fn_wp = @(pt) costfn_sqdist_value_pt_shapes(pt, rectangle_array, epsilon);
grad_c_obs_fn_wp = @(pt) costfn_sqdist_grad_pt_shapes(pt, rectangle_array, epsilon);
c_obs_fn_arclength_wpset = @(xi) value_arclength_fn_wp( xi, c_obs_fn_wp, p_start );
grad_c_obs_fn_arclength_wpset = @(xi) gradient_arclength_fn_wp( xi, c_obs_fn_wp, grad_c_obs_fn_wp, p_start);

c_final = @(xi) costfn_smooth_value_wpset(xi, A, b, c) + lambda*c_obs_fn_arclength_wpset( xi );
grad_final = @(xi) costfn_smooth_grad_wpset(xi, A, b) + lambda*grad_c_obs_fn_arclength_wpset( xi );

%% Create constraints
constraints = get_empty_constraints();

%% Chomp options
options = get_chomp_options();
options.Eta = 5000;
options.MinIter = 1;
options.MaxIter = 100;
options.TolFun = 1e-3;
options.Metric = A;
options.MetricInverse = inv(A);
options.DecreaseStepSize = 0;

%% Perform chomp
[ xi_best, cost_best, exitflag, output ] = covariant_gradient_descent( xi, c_final, grad_final, constraints, options );

%% Visualize
plot_chomp_history( output );
figure, plot_cost_history( output );

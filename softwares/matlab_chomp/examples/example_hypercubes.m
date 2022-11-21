%% 
% Copyright (c) 2016 Carnegie Mellon University, Sanjiban Choudhury <sanjibac@andrew.cmu.edu>
%
% For License information please see the LICENSE file in the root directory.
%
%% EXAMPLE_HYPERCUBES
% Perform chomp on 8 dimension hypercubes
clc;
clear;
close all;

%% Scenario
dim = 8;
centre = 0.4*ones(1,dim);
width =  0.2*ones(1,dim);

shapes_array(1) = get_hypercube_axis_aligned_shape(centre, width);

%% Create a trajectory
p_start = zeros(1,dim);
p_end = 1*ones(1,dim);
N = 100;
xi = get_wpset(p_start, p_end, N);

%% Create cost function
lambda = 100;

[ A, b, c ] = create_smooth_matrices( p_start, p_end, size(xi,1) - 2 );

epsilon = 0.05;
c_obs_fn_wp = @(pt) costfn_sqdist_value_pt_shapes(pt, shapes_array, epsilon);
grad_c_obs_fn_wp = @(pt) costfn_sqdist_grad_pt_shapes(pt, shapes_array, epsilon);
c_obs_fn_arclength_wpset = @(xi) value_arclength_fn_wp( xi, c_obs_fn_wp, p_start );
grad_c_obs_fn_arclength_wpset = @(xi) gradient_arclength_fn_wp( xi, c_obs_fn_wp, grad_c_obs_fn_wp, p_start);

c_final = @(xi) costfn_smooth_value_wpset(xi, A, b, c) + lambda*c_obs_fn_arclength_wpset( xi );
grad_final = @(xi) costfn_smooth_grad_wpset(xi, A, b) + lambda*grad_c_obs_fn_arclength_wpset( xi );

%% Create constraints
constraints = get_empty_constraints();

%% Chomp options
options = get_chomp_options();
options.Eta = 3000;
options.MinIter = 1;
options.MaxIter = 30;
options.TolFun = 1e-5;
options.Metric = A;
options.MetricInverse = inv(A);
options.DecreaseStepSize = 0;

%% Perform chomp
[ xi_best, cost_best, exitflag, output ] = covariant_gradient_descent( xi, c_final, grad_final, constraints, options );
figure, plot_cost_history( output );

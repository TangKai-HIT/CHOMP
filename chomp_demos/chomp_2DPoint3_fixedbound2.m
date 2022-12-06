%%CHOMP 2D point 3-order smoother (use 3-order metric, 1-order smoothness cost)
% Test CHOMP algorithm to plan a 2D point trajectory using 3-order differetial metric (full matrix) and 1-order
% smoothness cost function
clc; clear; close all;
addpath(genpath("..\my_util"), genpath("..\softwares\"), genpath("..\chomp\"));

%% Define & init 2D point trajectory
p_start = [0 0.5];
p_end = [1 0.5];
N = 200;
dt = 1/(N-1);
[p, pd, pdd, tSamples, ~]=trapveltraj([p_start', p_end'], N);

% figure("Position",[500,100, 800, 800]);
% subplot(3,1,1);
% plot(tSamples, pd); legend('x', 'y'); title("vel")
% subplot(3,1,2);
% pdd(:, end) = pdd(:, end-1);
% plot(tSamples, pdd); legend('x', 'y'); title("accel")
% subplot(3,1,3);
% pddd = diff(pdd, 1, 2)/dt; pddd = [pddd, pddd(:, end)];
% plot(tSamples, pddd); legend('x', 'y'); title("jerk")

%% Construct 2D cost map
bbox = [0 1 0 1]; %unit bounding box
% rectangle 1
rectangle_array(1).low = [0.2 0.2];
rectangle_array(1).high = [0.4 0.5];
% rectangle 2
rectangle_array(2).low = [0.6 0.5];
rectangle_array(2).high = [0.8 0.9];
% Get map
resolution = 0.001; %resolution of map
map = rectangle_maps( bbox, rectangle_array, resolution);
% Cost map 
epsilon = 0.12; % horizon upto which distance is computed
cost_map = create_costmap_sqdist(map, epsilon);
% Cost map derivatives
[cost_map_x, cost_map_y] = get_cost_map_derivatives(cost_map);
% Visualize cost map & robot
figure;
visualize_cost_map(cost_map);
visualize_rectangles(gca, rectangle_array, 'r', '--');

% hold on; plot(p(1, :), p(2, :), '-b', 'LineWidth', 1); 

%% Init chomp planner
chomp2D = chomp_wp_planner(p', 3, true, true);

%% Define cost function
lambda1 = 2;
lambda2 = 70;
[start_pts, end_pts] = chomp2D.get_boundary_pt();

% Get 3-order metric
[ A3, ~, ~, ~, ~ ] = get_smooth_matrices_full(chomp2D.num_pt , 3, ...
                             zeros(3,2), zeros(3,2)); % full smooth matrices

% Get 1-order smoothness cost parameters
[ A1, ~, b1, c1, ~ ] = get_smooth_matrices_full(chomp2D.num_pt , 1, ...
                        start_pts(end,:), end_pts(1,:)); % full smooth matrices

disFieldfn = @(xi) value_wpset_map( xi, cost_map ); % distance field evaluation function handle
disGradfn = @(xi) [value_wpset_map( xi, cost_map_x ), value_wpset_map( xi, cost_map_y )]; % distance field gradient evaluation function handle

cost_func = @(xi) lambda1 * costfn_smooth_wp(xi, A1, b1, c1, dt, 1) + lambda2 * cosfn_obs_wp({xi}, disFieldfn, {p_start}); 
grad_func = @(xi) lambda1 * grad_costfn_smooth_wp( xi, A1, b1 ,dt, 1) + lambda2 * grad_costfn_obs_wp({xi}, disFieldfn, disGradfn, [], dt, {p_start} );

chomp2D.set_cost_grad_fn(cost_func, grad_func);

%% Set options
chomp2D.options.Eta = 3*1e9; % for full smooth matrix case 
chomp2D.options.MinIter = 1;
chomp2D.options.MaxIter = 500;
chomp2D.options.TolFun = 1e-3;
chomp2D.options.MetricInverse = inv(A3);
chomp2D.options.Metric = A3;

%% Perform chomp
tic;
[optim_traj, cost_best, exitflag, output] = chomp2D.solve();
solved_time = toc; 
fprintf("solve time: %.4f (s)\n", solved_time);
fprintf("iterations: %d\n", output.iterations);
fprintf("exitFlag: %d\n", exitflag);

%% Visualize
chomp2D.deformHistory_plot(gca, @cool);
figure, plot_cost_history( output );

%% Plot velcity, acceleration, jerk
%boundary condition
% boundCondition{1} = [0,0];
% boundCondition{2} = [0,0; 0,0];
% boundCondition{3} = [0,0; 0,0; 0,0];
[vel_traj, accel_traj, jerk_traj] = get_vel_accel_jerk(optim_traj, dt);
%plot
t_sque = linspace(0, 1, N);
figure("Position",[500,100, 800, 800]);

subplot(3,1,1);
plot(t_sque, vel_traj); legend('x', 'y');
title("vel")
subplot(3,1,2);
plot(t_sque, accel_traj); legend('x', 'y');
title("accel")
subplot(3,1,3);
plot(t_sque, jerk_traj); legend('x', 'y');
title("jerk")
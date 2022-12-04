%%CHOMP 2D point: use CHOMP algorithm to plan a 2D point trajectory with
%%fixed boundary points
clc; clear; close all;
addpath(genpath("..\my_util"), genpath("..\softwares\"), genpath("..\chomp\"));

%% Define & init 2D point trajectory
p_start = [0 0.5];
p_end = [1 0.5];
N = 100;
dt = 1/(N-1);
[p, pd, pdd, tSamples, ~]=trapveltraj([p_start', p_end'], N);

% figure; plot(tSamples, p); title('position'); legend('x','y');
% figure; plot(tSamples, pd); title('velocity'); legend('dx','dy');
% figure; plot(tSamples, pdd); title('acceleration'); legend('ddx','ddy');
% pddd = diff(pdd, 1, 2)/dt; pddd = [pddd, pddd(:, end)];
% figure; plot(tSamples, pddd); title('jerk'); legend('ddx','ddy');

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
smooth_order = 1;
chomp2D = chomp_wp_planner(p', smooth_order, true, false);

%% Define cost function
lambda = 70;

[ A, K, b, c, e ] = get_smooth_matrices_part(chomp2D.num_pt , smooth_order, ...
                             p_start, p_end); % smooth matrices

disFieldfn = @(xi) value_wpset_map( xi, cost_map ); % distance field evaluation function handle
disGradfn = @(xi) [value_wpset_map( xi, cost_map_x ), value_wpset_map( xi, cost_map_y )]; % distance field gradient evaluation function handle

cost_func = @(xi) costfn_smooth_wp(xi, A, b, c, dt, smooth_order) + lambda * cosfn_obs_wp({xi}, disFieldfn, {p_start}); 
grad_func = @(xi) grad_costfn_smooth_wp( xi, A, b ,dt, smooth_order) + lambda * grad_costfn_obs_wp({xi}, disFieldfn, disGradfn, [], dt, {p_start} );

chomp2D.set_cost_grad_fn(cost_func, grad_func);

%% Set options
chomp2D.options.Eta = 5000;
chomp2D.options.MinIter = 1;
chomp2D.options.MaxIter = 100;
chomp2D.options.TolFun = 1e-3;
chomp2D.options.MetricInverse = inv(A);
chomp2D.options.Metric = A;

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
boundCondition{1} = [0,0];
boundCondition{2} = [0,0; 0,0];
boundCondition{3} = [0,0; 0,0; 0,0];
[vel_traj, accel_traj, jerk_traj] = get_vel_accel_jerk(optim_traj, dt, boundCondition);
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
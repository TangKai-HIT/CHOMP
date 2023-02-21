%%CHOMP test 2D: using a planar 3-R link to test CHOMP algorithm for trajectory planning
clc; clear; close all;
addpath(pwd, genpath("..\my_util"), genpath("..\softwares\"), genpath("..\chomp\"));

%% Define 3-R planar link
basePos = [0.05, 0.4]; %base position
initJointPos = [-pi/8, pi/8, pi/8];
linkLen  = [0.3, 0.3, 0.3];
%Sample body points for obstacle avoidance
link_Id = [2,3]; %which links to sample
num_samples = [2, 3]; % corresponding number of samples
%Init robot_3R_test
robot_3R_test = planarRobot_3R(linkLen, basePos);
robot_3R_test.addSamplePtOnLink(link_Id, num_samples);

robot_3R = construct3R_robot(initJointPos, linkLen, false);
robot_3R.DataFormat = 'row'; % row vector configuration
% show(robot_3R);
%Get IK
Ik = inverseKinematics('RigidBodyTree', robot_3R);

%% Construct 2D cost map
bbox = [0 1 0 1]; %unit bounding box
% rectangle 1
width = 0.1;
height = 0.1;
rectangle_array(1).low = [0.45 0.6]; %botton left
rectangle_array(1).high = [rectangle_array(1).low(1)+width, rectangle_array(1).low(2)+height]; %up right
resolution = 0.001; %resolution of map
% Get map
map = rectangle_maps( bbox, rectangle_array, resolution);
% Cost map 
epsilon = 0.1; % horizon upto which distance is computed
cost_map = create_costmap_sqdist(map, epsilon);
% Cost map derivatives
[cost_map_x, cost_map_y] = get_cost_map_derivatives(cost_map);

%% Define init trajectory
p_start = [0.25 0.65];
theta_start = 3*pi/4;
initpose_base = [p_start-basePos, theta_start];
initconfig = get_config2D_Ik(robot_3R, "endEffector", initpose_base, homeConfiguration(robot_3R), Ik);

p_end = [0.65 0.65];
theta_end = 1.5*pi/4;
endpose_base = [p_end-basePos, theta_end];
endconfig = get_config2D_Ik(robot_3R, "endEffector", endpose_base, homeConfiguration(robot_3R), Ik);

N = 200;
dt = 1/(N-1);
t_end = 1;
tSamples = 0:dt:t_end;
[pose, posed, posedd, ~] = quinticpolytraj([initpose_base ; endpose_base]', [0, t_end], tSamples); %end pose x,y,theta w.r.t base

%%Plot end trajectory
% figure("Position",[500,100, 800, 800]);
% subplot(3,1,1);
% plot(tSamples, pd(1:2,:)); legend('x', 'y'); title("vel")
% subplot(3,1,2);
% pdd(:, end) = pdd(:, end-1);
% plot(tSamples(1:end-1), pdd(1:2,1:end-1)); legend('x', 'y'); title("accel")
% subplot(3,1,3);
% pddd = diff(pdd(:,1:end-1), 1, 2)/dt; pddd = [pddd, pddd(:, end)];
% plot(tSamples(1:end-1), pddd(1:2, :)); legend('x', 'y'); title("jerk")

%% Initial visitualization
%Visualize baseic scene: cost map & robot
figure;
robot_3R_test.updateAllPtPosition(initJointPos);
plotMotionScene2D(gca, robot_3R_test, cost_map, rectangle_array, 'black', 1, true, "blue");
plot(gca, [p_start(1), p_end(1)], [p_start(2), p_end(2)], '--b', 'LineWidth', 1);

%Get configuration space trajectory
configTraj = get_configTraj2D_Ik(robot_3R, "endEffector", pose', initconfig, Ik);

%Plot trajectory color map
robot_3R_test.updateAllPtPosition(configTraj);
figure;
plotMotionScene2D(gca, robot_3R_test, cost_map, rectangle_array, flipud(gray(N/10)), 1:10:N);

%% Init chomp planner
chomp_3R = chomp_wp_planner(configTraj, 3, true); %3- order smoothness

%% Define cost function
lambda1 = 1;
lambda2 = 80;
[start_pts, end_pts] = chomp_3R.get_boundary_pt();

% Get 3-order metric
[ A3, ~, ~, ~, ~ ] = get_smooth_matrices_full(chomp_3R.num_pt , 3, ...
                             zeros(3,2), zeros(3,2)); % full 3-order smooth matrices

% Get 1-order smoothness cost parameters
[ A1, ~, b1, c1, ~ ] = get_smooth_matrices_full(chomp_3R.num_pt , 1, ...
                        start_pts(end,:), end_pts(1,:)); % full 1-order smooth matrices

disFieldfn = @(xi) value_wpset_map( xi, cost_map ); % distance field evaluation function handle
disGradfn = @(xi) [value_wpset_map( xi, cost_map_x ), value_wpset_map( xi, cost_map_y )]; % distance field gradient evaluation function handle

getJacobfn = @(xi) robot_3R_test.getSamplePtJacobian(); %sampled body points jacobian func handle
getSBTrajfn = @(xi) robot_3R_test.updateSamplePtPosition(xi); %sampled body points trajectory func handle
 
cost_func = @(xi) lambda1 * costfn_smooth_wp(xi, A1, b1, c1, dt, 1) + ...
                                    lambda2 * cosfn_obs_wp(getSBTrajfn(xi), disFieldfn); 

grad_func = @(xi) lambda1 * grad_costfn_smooth_wp( xi, A1, b1 ,dt, 1) + ...
                                    lambda2 * grad_costfn_obs_wp(getSBTrajfn(xi), disFieldfn, disGradfn, getJacobfn(xi), dt);

chomp_3R.set_cost_grad_fn(cost_func, grad_func);

%% Set options
chomp_3R.options.Eta = 3*1e9; %A1 metric: 5000, A3 metric: 3*1e9
chomp_3R.options.MinIter = 1;
chomp_3R.options.MaxIter = 500;
chomp_3R.options.TolFun = 1e-3;
chomp_3R.options.MetricInverse = inv(A3);
chomp_3R.options.Metric = A3;
chomp_3R.options.InequConstraintAlgorithm = 'qp_full'; %Choices {'qp_full',  'qp_sparse', 'qp_warmstart'}

%% Set constraint
config_lb = -pi*ones(chomp_3R.num_pt, 1);
config_ub = pi*ones(chomp_3R.num_pt, 1);
chomp_3R.constraints.lb = {config_lb, config_lb, config_lb};
chomp_3R.constraints.ub = {config_ub, config_ub, config_ub};

%% Perform chomp
tic;
[optim_traj, cost_best, exitflag, output] = chomp_3R.solve();
solved_time = toc; 
fprintf("solve time: %.4f (s)\n", solved_time);
fprintf("iterations: %d\n", output.iterations);
fprintf("exitFlag: %d\n", exitflag);

%% Visualize optimal trajectory
%update robot
robot_3R_test.updateAllPtPosition(optim_traj);
figure;
 plotMotionScene2D(gca, robot_3R_test, cost_map, rectangle_array, flipud(gray(N/10)), 1:10:N);

%% Visualize end point trajectory
% chomp_3R.deformHistory_plot(gca, @cool);
figure, plot_cost_history( output );

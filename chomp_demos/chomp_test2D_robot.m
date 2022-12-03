%%CHOMP test 2D: using a planar 3-R link to test CHOMP algorithm for trajectory planning
clc; clear;
addpath(genpath("..\my_util"), genpath("..\softwares\"), genpath("..\chomp\"));

%% Define 3-R planar link (kinematic)
basePos = [0, 0.5]; %base position
initJointPos = [-pi/8, pi/8, pi/8];
linkLen  = [0.2, 0.2, 0.2];
robot_3R = construct3R_robot(initJointPos, linkLen, false);
% show(robot_3R);

%% Construct 2D cost map
bbox = [0 1 0 1]; %unit bounding box
% rectangle 1
width = 0.1;
height = 0.1;
rectangle_array(1).low = [0.3 0.5]; %botton left
rectangle_array(1).high = [rectangle_array(1).low(1)+width, rectangle_array(1).low(2)+height]; %up right
resolution = 0.001; %resolution of map
% Get map
map = rectangle_maps( bbox, rectangle_array, resolution);
% Cost map 
epsilon = 0.1; % horizon upto which distance is computed
cost_map = create_costmap_sqdist(map, epsilon);
% Cost map derivatives
[cost_map_x, cost_map_y] = get_cost_map_derivatives(cost_map);
% Visualize cost map & robot
figure;
visualize_cost_map(cost_map);
visualize_rectangles(gca, rectangle_array, 'r', '--');
plotSerialRobot2D(gca, robot_3R, homeConfiguration(robot_3R), basePos);


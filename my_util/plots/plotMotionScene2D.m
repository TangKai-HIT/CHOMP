function  plotMotionScene2D(ax, robot, costmap, rectangle_array, robotColor, slice, plotsampFlag, samplecolor)

cla(ax); %clear axis

visualize_cost_map(costmap, ax);
visualize_rectangles(ax, rectangle_array, 'r', '--');

if exist("slice","var")
    robot.plotRobot2D(ax, robotColor, slice);
else
    robot.plotRobot2D(ax, robotColor);
end

if nargin>6
    if plotsampFlag
        robot.plotSampledPt(ax, samplecolor, slice);
    end
end
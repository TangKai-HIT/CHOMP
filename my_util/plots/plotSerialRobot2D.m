function plotSerialRobot2D(ax, robot, config, basePos)
%PLOTSERIALROBOT2D 
X = zeros(1,robot.NumBodies);
Y = zeros(1, robot.NumBodies);

hold on;
for i=1:robot.NumBodies
    trans = getTransform(robot,config,robot.BodyNames{i});
    X(i) = trans(1,4) + basePos(1);
    Y(i) = trans(2,4) + basePos(2);
    plot(ax, X(i), Y(i), 'o', 'MarkerSize',10, 'Color', 'black', 'MarkerFaceColor','black'); 

    if i>1
        plot(ax, X(i-1:i), Y(i-1:i), '-k', 'MarkerSize',6);
    end
end



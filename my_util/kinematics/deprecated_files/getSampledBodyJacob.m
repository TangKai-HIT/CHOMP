function bodyJacobian = getSampledBodyJacob(sampledRobot, namelist, configTraj, numBodyPts, taskSpaceDim)
%GETSAMPLEDBODYJACOB
bodyJacobian = cell(1,numBodyPts);
N = size(configTraj, 1);
configDim = size(configTraj, 2);

switch taskSpaceDim
    case 2
        taskSpaceId = [4,5];
    case 3
        taskSpaceId = 4:6;
end

Id = 1;
for i=1:length(namelist)
    for j = 1:length(namelist{i})
        bodyJacobian{Id} = zeros(taskSpaceDim, configDim, N); %init
        
        for n=1:N
            jacobian = geometricJacobian(sampledRobot,configTraj(n, :),namelist{i}{j});
            bodyJacobian{Id}(:, :, n) = jacobian(taskSpaceId, :);
        end

        Id =Id+1;
    end
end
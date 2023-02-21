function bodyPts = getSampledBodyTraj(sampledRobot, namelist, configTraj, basePos, numBodyPts)
%GETSAMPLEDBODYTRAJ
bodyPts = cell(1,numBodyPts);
N = size(configTraj, 1);
dim = length(basePos);

Id = 1;
for i=1:length(namelist)
    for j = 1:length(namelist{i})
        
        bodyPts{Id} = zeros(N, dim);
       
        for n=1:N
            trans = getTransform(sampledRobot, configTraj(n, :), namelist{i}{j});
            bodyPts{Id}(n, :) = trans(1:dim, 4)' + basePos;
        end

        Id =Id+1;
    end
end
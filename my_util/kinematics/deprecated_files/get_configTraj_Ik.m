function configTraj = get_configTraj_Ik(robot, name_EE, transMatrixArray, initConfigGuess, Ik)
%GET_CONFIGTRAJ_IK
%   robot
%   name_EE
%   transMatrixArray
%   initConfigGuess
%   Ik

if ~exist("Ik", "var")
    Ik = inverseKinematics('RigidBodyTree',robot);
end

N = length(transMatrixArray);
configTraj = zeros(N, length(initConfigGuess));

initialguess = initConfigGuess;
for i=1:length(transMatrixArray)
    [config,~] = Ik(name_EE, transMatrixArray{i}, [1e-3*ones(1,3), 1e-3*ones(1,3)], initialguess);

    configTraj(i, :) = config;
    initialguess = config;
end
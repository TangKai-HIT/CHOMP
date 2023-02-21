function configTraj = get_configTraj2D_Ik(robot, name_EE, poseArray, initConfigGuess, Ik)
%GET_CONFIGTRAJ2D_IK
%   robot
%   name_EE
%   poseArray: N X 3
%   initConfigGuess: 1 X3 row
%   Ik

if ~exist("Ik", "var")
    Ik = inverseKinematics('RigidBodyTree',robot);
end

N = size(poseArray, 1);
configTraj = zeros(N, 3);

initialguess = initConfigGuess;
for i=1:N
    config = get_config2D_Ik(robot, name_EE, poseArray(i, :), initialguess, Ik);

    configTraj(i, :) = config;
    initialguess = config;
end
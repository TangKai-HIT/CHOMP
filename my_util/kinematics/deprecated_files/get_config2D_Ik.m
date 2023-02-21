function config = get_config2D_Ik(robot, name_EE, pose, initConfigGuess, Ik)
%GET_CONFIG2D_IK
%   robot: DataFormat should be set to "row"
%   name_EE
%   pose: x y theta
%   initConfigGuess
%   Ik

if ~exist("Ik", "var")
    Ik = inverseKinematics('RigidBodyTree',robot);
end

T = eul2tform([pose(3),0,0],'ZYX');
T(1:2, 4) = pose(1:2);

[config,~] = Ik(name_EE, T, [1e-3*ones(1,3), 1e-3*ones(1,3)], initConfigGuess);
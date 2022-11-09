%%co-simulation with coppeliasim(stepping mode) :UR5 trajectory planning, tracking & get demonstrated trajectory data 
%%Note: corresponding co-sim scene is 'UR5_IK_trajTracking.ttt'
addpath(genpath(pwd)); 
                             
%% setup client, get objects
fprintf('Program started\n');

client = RemoteAPIClient();
sim = client.getObject('sim');

% enable the stepping mode on the client:
client.setStepping(true);

%% UR5 kinematic parameters
L(1) = 89.159*1e-3;
L(2) = 135.85*1e-3;
L(3) = 425*1e-3;
L(4) = 119.7*1e-3;
L(5) = 392.25*1e-3;
L(6) = 93*1e-3;
L(7) = 94.65*1e-3;
L(8) = 82.3*1e-3;

screwAxis = [L(3)+L(5)+L(7)              0                    0           0          0           0;
                     -(L(2)-L(4)+L(6))              0                    0           0        L(8)        0;
                                 0               L(3)+L(5)+L(7)  L(5)+L(7)    L(7)      0           0;
                                 0                            1                   1             1        0            0;
                                 0                            0                   0             0        1            0;
                                 -1                          0                   0              0        0            1];

tip = sim.getObject('/UR5[1]/Tip');
base = sim.getObject('/UR5[1]/O0');
target = sim.getObject('/Target');

UR5_kine = link_6R(screwAxis, Coppe_getMatrix2SE3(sim.getObjectMatrix(tip, base)));   

%% Get script handle
UR5_handle = sim.getObject('/UR5[1]');
UR5_script = sim.getScript(sim.scripttype_childscript, UR5_handle); %script handle

%% Start simulation
sim.startSimulation();
client.step(); %init script

%% Move to initial pose in a chosen path
PathNumber = 0;
pathHandle = sim.getObject(sprintf('/Path[%d]', PathNumber));

sim.callScriptFunction('moveToPathInitPose', UR5_script, PathNumber);
reachTime = sim.getSimulationTime(); %record time

while ~sim.callScriptFunction('checkReachCondition', UR5_script, target, 1e-3)
    client.step();
end
reachTime = sim.getSimulationTime() - reachTime;
fprintf('reached path[%d]\n', PathNumber);
fprintf('time of reaching: %.3f\n', reachTime);

%% Planning time optimal trajectory along the path
%get path pose, path parameter info & joint configuration along the path 
pathInfo = sim.callScriptFunction('getPathInfo', UR5_script, false, PathNumber);
[pathData, pathLen] = Coppe_pathInfoCovert(pathInfo);

jointsConfig = sim.callScriptFunction('getConfigAlongPath', UR5_script, PathNumber);
jointsData = Coppe_pointsSqueConvert(jointsConfig, 6);

%TOPPRA planning
velLimits = repmat([-pi/2,pi/2], 6, 1);
accelLimits = repmat([-pi, pi], 6, 1);
[jointsPosPlanned, jointsVelPlanned, jointsAccPlanned, t] = contopptraj(jointsData, velLimits, accelLimits, NumSamples=300);

%% Tracking the time optimal trajectory along the path
%create a dummy denoting end of trajectory
endPoint = sim.createDummy(0.01, zeros(12, 1));
sim.setObjectAlias(endPoint, sprintf('endPointInPath%d', PathNumber));
sim.setObjectPose(endPoint, pathHandle, pathData(:, end));

%get joints handle
jointsHandle = zeros(1,6);
for i =1:6
    jointsHandle(i) = sim.getObject(sprintf('/UR5[1]/UR5_joint%d', i));
end

%loop velocity output control using se3
% Kp = 1;
% Ki = 0.1;
% Xe = zeros(6,1);
% Xe_sum = zeros(6,1);

t_now = 0;  
time = sim.getSimulationTime();
disp('Start following path...')
while ~sim.callScriptFunction('checkReachCondition', UR5_script, endPoint, 1e-2)
    %%Control1: Configuaration Open Loop in task space
    if t_now<= t(end)
        interpPos = interp1(t, jointsPosPlanned', t_now, "linear"); %return 1 X 6
        interpVel = interp1(t, jointsVelPlanned', t_now, "linear"); %return 1 X 6
    
        for i =1:6
            sim.setJointTargetPosition(jointsHandle(i), interpPos(i));
            sim.setJointTargetVelocity(jointsHandle(i), interpVel(i));
        end
    else
         for i =1:6
            sim.setJointTargetPosition(jointsHandle(i), jointsPosPlanned(i, end));
            sim.setJointTargetVelocity(jointsHandle(i), jointsVelPlanned(i, end));
        end
    end
    
    client.step();
    t_now = sim.getSimulationTime() - time;
    %     [tipSE3, Jacob_b] = UR5_kine.updateFK(jointAngles);  
end
disp('Reached end of the path!')

%% Press any button to end simulation
disp('Press any button to end simulation...');
pause;

sim.stopSimulation();
fprintf('Program ended\n');
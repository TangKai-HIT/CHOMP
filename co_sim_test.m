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

tip = sim.getObject('/UR5[1]/O7');
UR5_kine = link_6R(screwAxis, Coppe_getMatrix2SE3(sim.getObjectMatrix(tip, sim.handle_world)));   

%% get script handle
UR5_handle = sim.getObject('/UR5[1]');
UR5_script = sim.getScript(sim.scripttype_childscript, UR5_handle); %script handle

%% start simulation, get time steps
sim.startSimulation();
client.step(); %init script

%move to initial pose in path[0] & record time
PathNumber = 0;
sim.callScriptFunction('moveToPathInitPose', UR5_script, PathNumber);
reachTime = sim.getSimulationTime(); %record time

while ~str2double(sim.getStringSignal('pose_reach_flag'))
    client.step();
    sim.callScriptFunction('checkReachCondition', UR5_script, nan);
end
reachTime = sim.getSimulationTime() - reachTime;
fprintf('time of reaching init pose:\n %.3f\n', reachTime);

pause;

sim.stopSimulation();
fprintf('Program ended\n');
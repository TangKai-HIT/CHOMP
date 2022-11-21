function robot = construct3R_robot(initJointPos, linkLen, showRobotDetail)
%CONSTRUCT3R_ROBOT  construct a 3R planar robot rigid body tree

robot = rigidBodyTree;

%define body 1
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('joint1','revolute');
jnt1.HomePosition = initJointPos(1);
tform = trvec2tform([0, 0, 0]); % User defined
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;

addBody(robot,body1,'base');

%define body 2
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('joint2','revolute');
jnt2.HomePosition = initJointPos(2); % User defined
tform2 = trvec2tform([linkLen(1), 0, 0]); % User defined
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2;
addBody(robot,body2,'body1'); % Add body2 to body1

%define body 3
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('joint3','revolute');
jnt3.HomePosition = initJointPos(3); % User defined
tform3 = trvec2tform([linkLen(2), 0, 0]); % User defined
setFixedTransform(jnt3,tform3);
body3.Joint = jnt3;
addBody(robot,body3,'body2'); 

%define end effector
endEffector = rigidBody('endEffector');
tform4 = trvec2tform([linkLen(3), 0, 0]); % User defined
setFixedTransform(endEffector.Joint, tform4);
addBody(robot,endEffector,'body3'); 

if showRobotDetail
    showdetails(robot);
end
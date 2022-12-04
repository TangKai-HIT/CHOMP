function [vel, accel, jerk] = get_vel_accel_jerk(p ,dt, boudaryCondition)
%GET_VEL_ACCEL_JERK get differetiation results of vel, accel, jerk
vel = diff(p, 1)/dt;
accel = diff(vel, 1)/dt;
jerk = diff(accel, 1)/dt;

if exist("boudaryCondition",'var')
    vel(end+1, :) = boudaryCondition{1}; % 1 pt at the end
    accel = [boudaryCondition{2}(1, :); accel; boudaryCondition{2}(2, :)]; % 1 pt at the start, 1 pt at the end
    jerk = [boudaryCondition{3}(1, :); jerk; boudaryCondition{3}(2:3, :)]; % 1 pt at the start, 2 pt at the end
else
    vel = [vel; vel(end, :)]; % 1 pt at the end
    accel = [accel(1, :); accel; accel(end, :)]; % 1 pt at the start, 1 pt at the end
    jerk = [jerk(1, :); jerk; repmat(jerk(end, :), 2, 1)]; % 1 pt at the start, 2 pt at the end
end
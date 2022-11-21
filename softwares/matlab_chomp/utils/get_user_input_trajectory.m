%% 
% Copyright (c) 2016 Carnegie Mellon University, Sanjiban Choudhury <sanjibac@andrew.cmu.edu>
%
% For License information please see the LICENSE file in the root directory.
%
%%
function xi = get_user_input_trajectory( p_start, p_goal, n )
%GET_USER_INPUT_TRAJECTORY Allow user to enter a trajectory through UI
%   p_start: start position
%   p_end: goal position
%   n: number of interpolated waypoints
%   xi: trajectory returned

title('Left click to enter points, right click to stop');
cntrl_pt = [p_start];
while(1)
    [x,y,button] = ginput(1);
    if (button == 3)
        break;
    end
    cntrl_pt = [cntrl_pt; x y];
end
cntrl_pt = [cntrl_pt; p_goal];
dcntrl_pt=diff(cntrl_pt,1,1);
d=sqrt(sum(dcntrl_pt.^2,2));
d=[0; cumsum(d)];
di=linspace(0,d(end),n);
xi=interp1(d,cntrl_pt,di);
title('');


end


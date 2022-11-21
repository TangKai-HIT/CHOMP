%% 
% Copyright (c) 2016 Carnegie Mellon University, Sanjiban Choudhury <sanjibac@andrew.cmu.edu>
%
% For License information please see the LICENSE file in the root directory.
%
%%
function constraints = get_goal_equality(pt, n)
%GET_GOAL_EQUALITY Get constraint struct corresponding to an equality for
%goal
%   pt: cell array of dim d for equality 
%   n: dimension of traj
%   constraint: standard constraint struct

constraints = get_empty_constraints();
for dim = 1:length(pt)
    constraints.Ceq{dim} = []; 
    constraints.deq{dim} = [];
    if (~isempty(pt{dim}))
        constraints.Ceq{dim} = [constraints.Ceq{dim}; [zeros(1,n-1) 1]];
        constraints.deq{dim} = [constraints.deq{dim}; pt{dim}]; 
    end
end

end


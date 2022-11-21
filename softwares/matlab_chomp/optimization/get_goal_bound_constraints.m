%% 
% Copyright (c) 2016 Carnegie Mellon University, Sanjiban Choudhury <sanjibac@andrew.cmu.edu>
%
% For License information please see the LICENSE file in the root directory.
%
%%
function constraints = get_goal_bound_constraints( lb, ub, n )
%GET_GOAL_BOUND_CONSTRAINTS Get constraint struct corresponding to a
%bounded goal
%   lb: lower bound of goal
%   ub: upper bound of goal
%   n: dimension of traj
%   constraint: standard constraint struct

constraints = get_empty_constraints();
for dim = 1:length(lb)
    constraints.C{dim} = []; 
    constraints.d{dim} = [];
    if (~isempty(lb{dim}))
        constraints.C{dim} = [constraints.C{dim}; [zeros(1,n-1) -1]];
        constraints.d{dim} = [constraints.d{dim}; -lb{dim}]; 
    end
    if (~isempty(ub{dim}))
        constraints.C{dim} = [constraints.C{dim}; [zeros(1,n-1) 1]];
        constraints.d{dim} = [constraints.d{dim}; ub{dim}];
    end
end

end


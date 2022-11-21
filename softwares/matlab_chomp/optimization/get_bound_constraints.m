%% 
% Copyright (c) 2016 Carnegie Mellon University, Sanjiban Choudhury <sanjibac@andrew.cmu.edu>
%
% For License information please see the LICENSE file in the root directory.
%
%%
function constraints = get_bound_constraints( lb, ub, n )
%GET_BOUND_CONSTRAINTS Returns a chomp constraint struct corresponding to
%bound constraints
%   lb: lowerbound
%   ub: upperbound
%   n: size of trajectory
%   constraints: constraint struct

constraints = get_empty_constraints();
for dim = 1:length(lb)
    constraints.C{dim} = []; 
    constraints.d{dim} = [];
    if (~isempty(lb{dim}))
        constraints.C{dim} = [constraints.C{dim}; -eye(n)];
        constraints.d{dim} = [constraints.d{dim}; -lb{dim}*ones(n,1)]; 
    end
    if (~isempty(ub{dim}))
        constraints.C{dim} = [constraints.C{dim}; eye(n)];
        constraints.d{dim} = [constraints.d{dim}; ub{dim}*ones(n,1)];
    end
end

end


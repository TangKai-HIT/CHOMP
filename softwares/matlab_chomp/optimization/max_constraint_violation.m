%% 
% Copyright (c) 2016 Carnegie Mellon University, Sanjiban Choudhury <sanjibac@andrew.cmu.edu>
%
% For License information please see the LICENSE file in the root directory.
%
%%
function viol = max_constraint_violation( xi, constraints )
%MAX_CONSTRAINT_VIOLATION Returns the maximum constraint violation
%   xi: trajectory
%   constraints: constraints struct
%   viol: max violation (scalar)

viol = -inf;
for dim = 1:length(constraints.C)
    if (~isempty(constraints.C{dim}))
        viol = max(viol, max(max(constraints.C{dim}*xi(:,dim) - constraints.d{dim})));
    end
end

end


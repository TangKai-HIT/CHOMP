%% 
% Copyright (c) 2016 Carnegie Mellon University, Sanjiban Choudhury <sanjibac@andrew.cmu.edu>
%
% For License information please see the LICENSE file in the root directory.
%
%%
function xi = constrained_update_equality( xi, grad, step_size, constraints, options )
%CONSTRAINED_UPDATE_EQUALITY CHOMP update step for equality constraints
%   xi: Initial trajectory (nxd matrix)
%   grad: Gradient (nxd matrix)
%   step_size: scalar step size
%   constraints: constraints structure
%   options: chomp options
%   xi: output traj

Minv = options.MetricInverse;

% Constrained update rule
for dim = 1:length(constraints.Ceq)
    Ceq = constraints.Ceq{dim};
    deq = constraints.deq{dim};

    if (~isempty(Ceq))
        P = step_size*(Ceq*Minv*Ceq');
        Q = (Ceq*xi(:,dim) - deq - step_size*Ceq*Minv*grad(:,dim));
        lambda_set{dim} = P \ Q;
    end
end

xi = xi - step_size*(Minv*(grad));
for dim = 1:length(constraints.Ceq)
    Ceq = constraints.Ceq{dim};
    if (~isempty(Ceq))
        xi(:,dim) = xi(:,dim) - step_size*Minv*(Ceq'*lambda_set{dim});
    end
end

end


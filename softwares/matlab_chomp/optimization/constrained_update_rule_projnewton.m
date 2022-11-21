%% 
% Copyright (c) 2016 Carnegie Mellon University, Sanjiban Choudhury <sanjibac@andrew.cmu.edu>
%
% For License information please see the LICENSE file in the root directory.
%
%%
function [xi, lambda_set] = constrained_update_rule_projnewton( xi, grad, lambda_set, step_size, constraints, options )
%CONSTRAINED_UPDATE_RULE_PROJNEWTON Update for chomp using dual projected
%newton
%   xi: Initial trajectory (nxd matrix)
%   grad: Gradient (nxd matrix)
%   lambda_set: cell array of lagrange multipliers
%   step_size: scalar step size
%   constraints: constraints structure
%   options: chomp options
%   xi: output traj
%   lambda_set: updated lagrange multipliers

Minv = options.MetricInverse;

% Constrained update rule
for dim = 1:length(constraints.C)
    C = constraints.C{dim};
    d = constraints.d{dim};

    if (~isempty(C))
        P = step_size*(C*Minv*C');
        Q = -(C*xi(:,dim) - d - step_size*C*Minv*grad(:,dim));
        if (isempty(lambda_set{dim}))
            lambda_set{dim} = zeros(size(d,1),1);
        end
        lambda_set{dim} = box_projected_newton_single_step( P, Q, lambda_set{dim}, zeros(size(d,1),1), inf*ones(size(d,1),1), options);
    end
end

xi = xi - step_size*(Minv*(grad));
for dim = 1:length(constraints.C)
    C = constraints.C{dim};
    if (~isempty(C))
        xi(:,dim) = xi(:,dim) - step_size*Minv*(C'*lambda_set{dim});
    end
end

end


%% 
% Copyright (c) 2016 Carnegie Mellon University, Sanjiban Choudhury <sanjibac@andrew.cmu.edu>
%
% For License information please see the LICENSE file in the root directory.
%
%%
function xi = constrained_update_qp( xi, grad, step_size, constraints, options )
%CONSTRAINED_UPDATE_QP Constrained inequality update using full QP
%   xi: Initial trajectory (nxd matrix)
%   grad: Gradient (nxd matrix)
%   step_size: scalar step size
%   constraints: constraints structure
%   options: chomp options
%   xi: output traj

M = options.Metric;
for dim = 1:length(constraints.C)
    C = constraints.C{dim};
    d = constraints.d{dim};
    opts = optimoptions('quadprog','Display','none','TolCon',options.TolCon);
    xi(:,dim) = quadprog((1/step_size)*M, grad(:,dim) - (1/step_size)*M*xi(:,dim), C, d, [], [], [], [],xi(:,dim), opts);
end
        
end


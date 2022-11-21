%% 
% Copyright (c) 2016 Carnegie Mellon University, Sanjiban Choudhury <sanjibac@andrew.cmu.edu>
%
% For License information please see the LICENSE file in the root directory.
%
%%
function x = box_projected_newton_single_step( P, Q, x0, lb, ub, options )
%BOX_PROJECTED_NEWTON_SINGLE_STEP Single update step for projected newton
%method
%   P: The quadratic term
%   Q: The linear term
%   x0: The initial guess
%   lb: Lower bound vector
%   ub: Upper bound vector
%   options: projected newton options
%   x: one step update

eps = options.TolCon;
step = options.ProjNewtStepSize;
x = x0;

grad = P*x + Q;
B = ((x <= lb + eps) & (grad > 0)) | ...
    ((x >= ub - eps) & (grad < 0));

F = ~B;

Pf = P(F,F);
if (~isempty(Pf))
    x(F) = x(F) - step * (Pf \ grad(F));
end
x(B) = x(B) - step * grad(B);

x = min(ub, max(lb, x));
end


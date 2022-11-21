%% 
% Copyright (c) 2016 Carnegie Mellon University, Sanjiban Choudhury <sanjibac@andrew.cmu.edu>
%
% For License information please see the LICENSE file in the root directory.
%
%%
function [ xi_best, cost_best, exitflag, output ] = covariant_gradient_descent( xi, cost_fn, grad_fn, constraints, options )
%COVARIANT_GRADIENT_DESCENT A general framework for covariant gradient
%descent
%   xi: initial trajectory (including end points)
%   cost_fn: a function handle that maps a trajectory to cost
%   grad_fn: a function handle that maps a trajectory to gradient of cost
%   constraints: chomp constraint struct containing equality / inequality
%   options: chomp options struct
%   xi_best: the feasible trajectory with least cost
%   cost_best: cost of xi_best
%   exit_flag: indication of why chomp terminated
%   output: output struct packing chomp analysis variables

%% Extract start ang goal
p_start = xi(1,:);
p_goal = xi(end,:);

xi(1,:) = [];
xi(end,:) = [];

if (options.FreeEndPoint)
    p_goal = [];
end

%% Setup
if (options.DecreaseStepSize)
    step_size = @(i) (1/options.Eta)*(1/sqrt(i));
else
    step_size = @(i) (1/options.Eta);
end
Minv = options.MetricInverse;
lambda_set = cell(1, length(constraints.C));

output.iterations = 0;
output.history = [];

traj.xi = [p_start; xi; p_goal];
traj.grad = grad_fn(xi);
traj.cost = cost_fn(xi);
traj.constraint_viol = max_constraint_violation( xi, constraints );
output.history = [output.history; traj];

%% Optimization
i = 1;
while (1)
    grad = traj.grad;

    % Update rule
    if (isempty(constraints.C) && isempty(constraints.Ceq))
        % Unconstrained update rule
        xi = xi - step_size(i)*(Minv*(grad));
    elseif (isempty(constraints.C))
        % Constrained update rule for equality only
        xi = constrained_update_equality( xi, grad, step_size(i), constraints, options );
    elseif (isempty(constraints.Ceq))
        % Constrained update rule for inequality
        if (strcmp(options.ConstraintAlgorithm, 'proj_newton'))
            [xi, lambda_set] = constrained_update_rule_projnewton( xi, grad, lambda_set, step_size(i), constraints, options );   
        elseif (strcmp(options.ConstraintAlgorithm, 'qp'))
            xi = constrained_update_qp( xi, grad, step_size(i), constraints, options );
        end
    end   
    
    % Saving intermediate trajectories
    traj.xi = [p_start; xi; p_goal];
    traj.grad = grad_fn(xi);
    traj.cost = cost_fn(xi);
    traj.constraint_viol = max_constraint_violation( xi, constraints );
    
    output.history = [output.history; traj];

    % Termination criteria
    if (i >= options.MinIter)
        if (abs(output.history(i).cost - output.history(i+1).cost)/output.history(i).cost < options.TolFun && ...
            output.history(i+1).constraint_viol < options.TolCon)
            exitflag = 1;
            break;
        end
    end
    
    if (i >= options.MaxIter)
        exitflag = 0;
        break;
    end
    
    i = i+1;
end

%% Extract best trajectory
valid_idx = extractfield(output.history, 'constraint_viol') < options.TolCon;
valid_traj = output.history(valid_idx);
[~, best_idx] = min(extractfield(valid_traj, 'cost'));
xi_best = valid_traj(best_idx).xi;
cost_best = valid_traj(best_idx).cost;
output.iterations = i;
end





function [ xi_best, cost_best, exitflag, output ] = chomp_QP( xi, bound_Id, cost_fn, grad_fn, constraints, options )
%CHOMP_QP perform QP in every step
%solve the quadratic problem with linearized equality and inequality constraint

%   xi: initial trajectory (including end points)
%   bound_Id:  index of boundary way points
%   cost_fn: a function handle that maps a trajectory to cost
%   grad_fn: a function handle that maps a trajectory to gradient of cost
%   constraints: chomp constraint struct containing equality / inequality
%   options: chomp options struct
%   xi_best: the feasible trajectory with least cost
%   cost_best: cost of xi_best
%   exit_flag: indication of why chomp terminated (1-converged; 0-reached max iterations; -1-failed, inequality constraint detected)
%   output: output struct packing chomp analysis variables

%% Extract start ang goal
p_start = xi(bound_Id{1}, :);

if (options.FreeEndPoint)
    p_goal = [];
else
    p_goal = xi(bound_Id{2}, :);
    xi(bound_Id{2}, :) = [];
end
xi(bound_Id{1}, :) = [];

%% Basic Setup
numDim = size(xi, 2); %dimension

if (options.DecreaseStepSize)
    step_size = @(i) (1/options.Eta)*(1/sqrt(i));
else
    step_size = @(i) (1/options.Eta);
end
M = options.Metric;

output.iterations = 0;
output.history = [];

traj.xi = [p_start; xi; p_goal];
traj.grad = grad_fn(xi);
traj.cost = cost_fn(xi);

output.history = [output.history; traj];

%% Method Setup
method = options.InequConstraintAlgorithm;
opts = optimoptions('quadprog','Display','none','TolCon',options.TolCon, 'Algorithm', 'interior-point-convex'); %defalut algorithm

if strcmpi(method, 'qp_sparse')  %use interior method for sparse matrix
    M = sparse(M); 
    
elseif strcmpi(method, 'qp_warmstart') %use warm start for active set method
    opts.Algorithm = 'active-set';
    warmstart = cell(1,numDim);
    for i=1:numDim
        warmstart{i} = optimwarmstart(xi(:, i),opts);
    end
end

%% Optimization
i = 1;
while (1)
    grad = traj.grad;

    %solve QP
    for dim = 1:numDim
        C =[]; d=[]; Ceq =[]; deq=[]; lb=[]; ub=[];

        if ~isempty(constraints.C)
            C = constraints.C{dim};
            d = constraints.d{dim};
        end
        if ~isempty(constraints.Ceq)
            Ceq = constraints.Ceq{dim};
            deq = constraints.deq{dim};
        end
        if ~isempty(constraints.lb)
            lb = constraints.lb{dim};      
        end
        if ~isempty(constraints.ub)
            ub = constraints.ub{dim};      
        end
        
        if strcmpi(method, 'qp_warmstart') %use warm start for active set method
            [warmstart{dim}, ~, ~, ~, ~] = quadprog((1/step_size(dim))*M, grad(:,dim) - (1/step_size(dim))*M*xi(:,dim), C, d, Ceq, deq, lb, ub, warmstart{dim});
            xi(:,dim) = warmstart{dim}.X;
        else
            xi(:,dim) = quadprog((1/step_size(dim))*M, grad(:,dim) - (1/step_size(dim))*M*xi(:,dim), C, d, Ceq, deq, lb, ub, xi(:,dim), opts);
        end
  
    end
    
    % Saving intermediate trajectories
    traj.xi = [p_start; xi; p_goal];
    traj.grad = grad_fn(xi);
    traj.cost = cost_fn(xi);
    
    output.history = [output.history; traj];

    % Termination criteria
    if (i >= options.MinIter)
        if (abs(output.history(i).cost - output.history(i+1).cost)/output.history(i).cost < options.TolFun)
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
[~, best_idx] = min(extractfield(output.history, 'cost'));
xi_best = output.history(best_idx).xi;
cost_best = output.history(best_idx).cost;
output.iterations = i;
end


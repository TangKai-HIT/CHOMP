function [ xi_best, cost_best, exitflag, output ] = chomp_CovGradDescent( xi, bound_Id, cost_fn, grad_fn, constraints, options )
%CHOMP_COVGRADDESCENT perform covariant gradient descent to iteratively
%solve the quadratic problem with linearized equality constraint 

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

%% Setup
if (options.DecreaseStepSize)
    step_size = @(i) (1/options.Eta)*(1/sqrt(i));
else
    step_size = @(i) (1/options.Eta);
end
Minv = options.MetricInverse;

output.iterations = 0;
output.history = [];

traj.xi = [p_start; xi; p_goal];
traj.grad = grad_fn(xi);
traj.cost = cost_fn(xi);

if ~isempty(constraints.Ceq)
    numDim = length(constraints.Ceq);
    lambda_set = cell(numDim); %init
end

output.history = [output.history; traj];

%% Optimization
i = 1;
while (1)
    grad = traj.grad;

    % Update rule
    if (isempty(constraints.C) && isempty(constraints.Ceq))
        % Unconstrained update rule
        xi = xi - step_size(i)*(Minv*(grad));
        
    elseif (~isempty(constraints.Ceq)) && isempty(constraints.C)
        % Constrained update rule for equality only
        for dim = 1:numDim
            Ceq = constraints.Ceq{dim};
            deq = constraints.deq{dim};
        
            if (~isempty(Ceq))
                P = step_size(i)*(Ceq*Minv*Ceq');
                Q = (Ceq*xi(:,dim) - deq - step_size(i)*Ceq*Minv*grad(:,dim)); % use xi_i
                lambda_set{dim} = P \ Q;
            end
        end
        
        xi = xi - step_size(i)*(Minv*(grad)); % update  xi_{i+1}
        for dim = 1:numDim
            Ceq = constraints.Ceq{dim};
            if (~isempty(Ceq))
                xi(:,dim) = xi(:,dim) - step_size(i)*Minv*(Ceq'*lambda_set{dim});
            end
        end

    else 
        exitflag = -1;
        disp("inequality constraint not supported! please use other optimization functions.")
        break;

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


function constraints = set_goal_boxconstraints( constraints, lb, ub, n )
%ADD_GOAL_BOXCONSTRAINTS set goal box constraints
%   lb: lowerbound ( 1 X dim array)
%   ub: upperbound ( 1 X dim array)
%   n: size of trajectory
%   constraints: constraint struct
dim = size(lb, 2);

if isempty(constraints.lb)
    constraints.lb = cell(1,dim);
end

if isempty(constraints.ub)
    constraints.ub = cell(1,dim);
end

for i=1:dim
    if isempty(constraints.lb{i})
        constraints.lb{i} = -inf * ones(n, 1);
    end

    if isempty(constraints.ub{i})
        constraints.ub{i} = inf * ones(n, 1);
    end

    constraints.lb{i}(end) = lb(i);
    constraints.ub{i}(end) = ub(i);
end
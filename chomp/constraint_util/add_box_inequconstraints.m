function constraints = add_box_inequconstraints( constraints, lb, ub, n )
%ADD_BOX_INEQUCONSTRAINTS add box constraints to the inequality part of a chomp constraint struct 
%   lb: lowerbound ( 1 X dim cell array)
%   ub: upperbound ( 1 X dim cell array)
%   n: size of trajectory
%   constraints: constraint struct

constraints.lb = lb;
constraints.ub = ub;

for dim = 1:length(lb)
    constraints.C{dim}=[];
    constraints.d{dim}=[];
    
    if (~isempty(lb{dim}))
        constraints.C{dim} = [constraints.C{dim}; -eye(n)];
        constraints.d{dim} = [constraints.d{dim}; -lb{dim}.*ones(n,1)]; 
    end
    if (~isempty(ub{dim}))
        constraints.C{dim} = [constraints.C{dim}; eye(n)];
        constraints.d{dim} = [constraints.d{dim}; ub{dim}.*ones(n,1)];
    end
end

end
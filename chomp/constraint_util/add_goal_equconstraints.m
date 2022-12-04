function constraints = add_goal_equconstraints(constraints, pt, n)
%ADD_GOAL_EQUCONSTRAINTS add equality constraints for goal to the constraint struct  
%   constraint: standard constraint struct
%   pt: cell array of dim d for equality 
%   n: dimension of traj

for dim = 1:length(pt)
    constraints.Ceq{dim} = []; 
    constraints.deq{dim} = [];
    if (~isempty(pt{dim}))
        constraints.Ceq{dim} = [constraints.Ceq{dim}; [zeros(1,n-1) 1]];
        constraints.deq{dim} = [constraints.deq{dim}; pt{dim}]; 
    end
end

end
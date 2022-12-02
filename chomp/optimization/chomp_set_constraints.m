function constraints = chomp_set_constraints(C, d, Ceq, deq, lb, ub)
%CHOMP_SET_CONSTRAINTS Returns a chomp constraint struct corresponding to
%different types of constraints
%   C: inequality constraint matrix (1 X dim cell array: {nXn} X dim)
%   d: inequalities (1 X dim cell array: {nX1} X dim)
%   Ceq: equality constraint matrix 
%   deq: equalities 
%   lb: lowerbound ( 1 X dim cell array)
%   ub: upperbound ( 1 X dim cell array)
%   constraints: constraint struct

if nargin == 7
    constraints.C = C;
    constraints.d = d;
    constraints.Ceq = Ceq;
    constraints.deq = deq;
%     constraints.lb = num2cell(lb, 1, ones(1,dim));
%     constraints.ub = num2cell(ub, 1, ones(1,dim));
    constraints.lb = lb;
    constraints.ub = ub;
else
    constraints.C = [];
    constraints.d = [];
    constraints.Ceq = [];
    constraints.deq = [];
    constraints.lb = [];
    constraints.ub = [];
end

end
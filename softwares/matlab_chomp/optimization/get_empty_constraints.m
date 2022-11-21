%% 
% Copyright (c) 2016 Carnegie Mellon University, Sanjiban Choudhury <sanjibac@andrew.cmu.edu>
%
% For License information please see the LICENSE file in the root directory.
%
%%
function constraints = get_empty_constraints()
%GET_EMPTY_CONSTRAINTS Get blank chomp constraint struct
%   constraint: struct containing
%   C: inequality constraint matrix
%   d: inequalities
%   Ceq: equality constraint matrix
%   deq: equalities

constraints.C = [];
constraints.d = [];
constraints.Ceq = [];
constraints.deq = [];
end


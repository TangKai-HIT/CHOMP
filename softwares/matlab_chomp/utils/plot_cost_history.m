%% 
% Copyright (c) 2016 Carnegie Mellon University, Sanjiban Choudhury <sanjibac@andrew.cmu.edu>
%
% For License information please see the LICENSE file in the root directory.
%
%%
function plot_cost_history( output )
%PLOT_COST_HISTORY Plots the cost versus iteration
%   output: chomp output struct

cost_hist = extractfield(output.history, 'cost');
plot(cost_hist);
title('Cost History');
xlabel('Iterations');
ylabel('Cost');
end


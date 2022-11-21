%% 
% Copyright (c) 2016 Carnegie Mellon University, Sanjiban Choudhury <sanjibac@andrew.cmu.edu>
%
% For License information please see the LICENSE file in the root directory.
%
%%
function plot_chomp_history( output )
%PLOT_CHOMP_HISTORY plots the traj history
%   output: chomp output struct

hold on;
cc=winter(length(output.history));
for i = 1:length(output.history)
    plot(output.history(i).xi(:,1), output.history(i).xi(:,2),'color',cc(i,:));
end

end


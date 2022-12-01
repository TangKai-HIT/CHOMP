function cost = costfn_smooth_wp( xi, A, b, c , dt)
%COSTFN_SMOOTH_WP Calculate smoothness cost of a trajectory (way point parameterization)
%   xi: discretized waypoint trajectory
%   A: smoothness quadratic matrix
%   b: smoothness linear matrix
%   c: smoothness constant matrix
%   cost: smoothness cost
%   dt: sample time

cost = 1/dt * trace(0.5*xi'*A*xi + xi'*b + c);

end
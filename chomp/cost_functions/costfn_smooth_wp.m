function cost = costfn_smooth_wp( xi, A, b, c , dt, smooth_order)
%COSTFN_SMOOTH_WP Calculate smoothness cost of a trajectory (way point parameterization)
%   xi: discretized waypoint trajectory
%   A: smoothness quadratic matrix
%   b: smoothness linear matrix
%   c: smoothness constant matrix
%   cost: smoothness cost
%   dt: sample time
%   smooth_order: order of smoothness (1~3)

cost = (1/dt)^(2*smooth_order - 1) * trace(0.5*xi'*A*xi + xi'*b + c);

end
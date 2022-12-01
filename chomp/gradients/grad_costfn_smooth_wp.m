function grad = grad_costfn_smooth_wp( xi, A, b ,dt)
%GRAD_COSTFN_SMOOTH_WP Calculate smoothness gradient
%   xi: discretized waypoint trajectory
%   A: smoothness quadratic matrix
%   b: smoothness linear matrix
%   cost: smoothness cost
%   dt: sample time

grad = 1/dt *(A*xi+b); 
end
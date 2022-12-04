function grad = grad_costfn_smooth_wp( xi, A, b ,dt, smooth_order)
%GRAD_COSTFN_SMOOTH_WP Calculate smoothness gradient
%   xi: discretized waypoint trajectory
%   A: smoothness quadratic matrix
%   b: smoothness linear matrix
%   cost: smoothness cost
%   dt: sample time
%   smooth_order: order of smoothness (1~3)

grad = (1/dt)^(2*smooth_order - 1) *(A*xi+b); 
end
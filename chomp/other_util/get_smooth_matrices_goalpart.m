function [ A, K, b, c, e ] = get_smooth_matrices_goalpart(n , order, p_init)
%GET_SMOOTH_MATRICES_GOALPART Create goal points partitioned matrices to compute 
%smooth cost function (forward differetiation)
%   *note*: way point xi is n X dim
%   p_init: points for initial differetiation at i=1 (1 X dim)
%   p_end: points for end differetiation at i=n (1 X dim)
%   n: number of points of trajectory (exclude start and end points)
%   A: smoothness quadratic matrix (n X n)
%   b: smoothness linear matrix (n X dim)
%   c: smoothness constant matrix (dim X dim)
%   K: differentiation matrix (n+order X n)
%   e: error matrix (n+order X dim)
%   order: order of differentiation (1~3)

%% Set up smoothness matrices
% f = 0.5*||Kxi + e||^2 = 0.5*A'*xi*A+xi'b+c

[ ~, K, ~, ~, e ] = get_smooth_matrices_full(n, order, p_init, zeros(size(p_init)));

%remove end points boundaries
K = K(1:end-order, :); 
e = e(1:end-order, :);

A = K'*K;
b = K'*e;
c = 0.5*(e'*e);

end
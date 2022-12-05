function [ A, K, b, c, e ] = get_smooth_matrices_part(n , order, p_init, p_end)
%GET_SMOOTH_MATRICES_PART Create partitioned matrices to compute smooth cost function (forward differetiation)
%   *note: way point xi is n X dim
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

[ ~, K, ~, ~, ~ ] = get_smooth_matrices_full(n , order);

switch order
    case 1
        E_0  = -1 * p_init;
        E_d  = 1 * p_end;
        numRows = n+1;

    case 2
        E_0  = 1 * p_init;
        E_d  = 1 * p_end;
        K = K(2:end-1, :);
        numRows = n;

    case 3
        E_0  = -1 * p_init;
        E_d  = -3 * p_end;
        K = K(3:end-2, :);
        numRows = n;

end

e = [E_0; zeros(numRows-2, size(p_init,2)); E_d];

A = K'*K;
b = K'*e;
c = 0.5*(e'*e);

end
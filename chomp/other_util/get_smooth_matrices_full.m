function [ A, K, b, c, e ] = get_smooth_matrices_full(n , order, p_init, p_end)
%GET_SMOOTH_MATRICES_FULL Create full matrices to compute smooth cost function (forward differetiation)
%   *note: way point xi is n X dim
%   p_init: points for initial differetiation at i=1 (order X dim)
%   p_end: points for end differetiation at i=n (order X dim)
%   n: number of points of trajectory (exclude start and end points)
%   A: smoothness quadratic matrix (n X n)
%   b: smoothness linear matrix (n X dim)
%   c: smoothness constant matrix (dim X dim)
%   K: differentiation matrix (n+order X n)
%   e: error matrix (n+order X dim)
%   order: order of differentiation (1~3)

%% Set up smoothness matrices
% f = 0.5*||Kxi + e||^2 = 0.5*A'*xi*A+xi'b+c
e = [];
b=[];
c=[];

K = zeros(n+order, n);

switch order
    case 1
        fd = [-1 1 zeros(1, n-2)];
        T = 1;
        sign = -1;

    case 2
        fd = [1 -2 1 zeros(1, n-3)];
        T = diag([1,1]);
        T(2,1) = -2;
        sign = 1;

    case 3
        fd = [-1 3 -3 1 zeros(1, n-4)];
        T = diag([1,1,1]);
        T(2,1) = -3; T(3,2) = -3;
        T(3,1)=3;
        sign = -1;
end

for i = 1:(n - order)
    K(i+order,:) = circshift(fd, [0 (i-1)]);
end

K(1:order, 1:order) = T;
K(n+1:n+order, n-order+1:n) = -T';
A = K'*K;

if nargin == 4
    E_0 = sign* T' * p_init;
    E_d = T * p_end;
    e = [E_0; zeros(n-order, size(p_init,2)); E_d];
    b = K'*e;
    c = 0.5*(e'*e);
end

end
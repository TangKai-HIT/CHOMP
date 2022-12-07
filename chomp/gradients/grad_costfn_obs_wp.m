function grad = grad_costfn_obs_wp( body_pt, disFieldfn, disGradfn, body_jacobian, dt, body_p_start )
%GRAD_COSTFN_OBS_WP Get gradient of obstacle cost functions
%   body_pt: n x ptDim way points set on sampled body points (1 X B cell array {n X ptDim})
%   disFieldfn: function handle for evaluating distance field on way points (func_handle @(xi) => n X 1)
%   disGradfn: function handle for evaluating distance field gradient on way points (func_handle @(xi)  => n X ptDim )
%   body_jacobian: jacobian matrices of body points w.r.t robot config (1 X B cell array {ptDim X configDim X N} or [])
%   dt: sample time
%   body_p_start: (optional) start coordinate of body way points (1 X B cell array)
%   grad: gradient of trajectory (way point) (n X configDim array)

N = size(body_pt{1}, 1);
ptDim = size(body_pt{1}, 2); %euclidean point dimension

grad = 0;
for i=1:length(body_pt)
    if (nargin <= 5)
        body_p_start{i} = body_pt{i}(1,:);
    end

    c = disFieldfn(body_pt{i}); % n X 1
    delta_c = disGradfn(body_pt{i}); % n X ptDim
    
    n = 1/dt;
    xi = body_pt{i};
    xi_d = n*diff([body_p_start{i}; xi]);
    xi_d_norm = normr(xi_d);
    xi_dd = n*([0 0; diff(xi_d)]);
    
    kappa = repmat((1./sum((xi_d).^2,2)), [1 ptDim]).*(xi_dd - xi_d_norm.*repmat(sum(xi_dd.*xi_d_norm, 2), [1 ptDim]));
    
    grad_i = dt.*repmat(sqrt(sum((xi_d).^2,2)), [1 ptDim]) .* ( delta_c - xi_d_norm.*repmat(sum(delta_c.*xi_d_norm,2), [1 ptDim])  - repmat(c, [1 ptDim]).*kappa);
    grad_i(isnan(grad_i) | isinf(grad_i)) = 0;

    %Jacobian part
    if ~isempty(body_jacobian) 
        configDim = size(body_jacobian{i}, 2);

        temp_grad_i = grad_i;
        grad_i = zeros(N, configDim);

        for n=1:N
            grad_i(n, :) = ((body_jacobian{i}(:, :, n))' * temp_grad_i(n, :)');
        end
    end
    
    %body point sum
    grad = grad + grad_i;
end

end
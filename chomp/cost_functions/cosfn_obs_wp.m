function value = cosfn_obs_wp( body_pt, disFieldfn, body_p_start )
%COSFN_OBS_WP obstacle avoidance cost function on way points
%   body_pt: n x dim way points set on sampled body points (1 X B cell array)
%   disFieldfn: function handle for evaluating distance field on a way point (func_handle @(xi) => n X 1)
%   body_p_start: (optional) start coordinate of body way points (1 X B cell array)

value = 0;
for i=1:length(body_pt)
    if (nargin <= 2)
        body_p_start{i} = body_pt{i}(1,:);
    end
    value = value + sum(sqrt(sum((diff([body_p_start{i}; body_pt{i}])).^2,2)) .* disFieldfn(body_pt{i}));
end

end
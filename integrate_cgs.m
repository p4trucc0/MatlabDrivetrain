function xi = integrate_cgs(xp, xd, dt, param)

if nargin < 4
    param = struct();
    param.protect_sign_change = true;
    param.max_abs_vals = 2000*ones(size(xp));
end

xi = xp + xd.*dt;
if param.protect_sign_change
    ss = sign(xi.*xp) < 0;
    xi(ss) = 0.0;
end

xi(abs(xi) > param.max_abs_vals) = param.max_abs_vals(abs(xi) > param.max_abs_vals).*sign(xi(abs(xi) > param.max_abs_vals));


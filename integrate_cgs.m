function xi = integrate_cgs(xp, xd, dt, param)
% Patrucco, 2020
% Integrate Cum Grano Salis
% To avoid most common sources of numerical instability the "wrong" (and
% efficient) way - the right one would be to decrease step size.


if nargin < 4
    param = struct();
    param.protect_sign_change = true;
    param.protect_clutch_sign_change = true;
    param.max_abs_vals = 2000*ones(size(xp));
end

% Initial guess
xi = xp + xd.*dt;

% Useful for wheels that'll be forced to lock before moving in the opposite
% direction.
if param.protect_sign_change
    ss = sign(xi.*xp) < 0;
    xi(ss) = 0.0;
end

% Avoid engine and clutch braking against each other in an unstable way.
if param.protect_clutch_sign_change
    if (((xp(1) > xp(2)) && (xi(2) > xi(1))) || ((xp(1) < xp(2)) && (xi(2) < xi(1))))
        xi(1) = xi(2);
    end
end

% Avoid excessive values.
xi(abs(xi) > param.max_abs_vals) = param.max_abs_vals(abs(xi) > param.max_abs_vals).*sign(xi(abs(xi) > param.max_abs_vals));


function w1v = differential_function(w0v, Jm, Jl, Jr, Mm, Ml, Mr, diff_mode, varargin)
% Simulates an open (0) or closed diff (1)

if isempty(varargin)
    frac = [.5 .5];
else
    frac = varargin{1};
end

tol = .1; % tolerance on const. bond

wm = w0v(1);
wl = w0v(2);
wr = w0v(3);

% Check constiutional bond
if (abs(wl + wr - 2*wm) > tol)
    warning('Differential is NOT respecting base hypothesis (sum of speeds). Returning zeros');
    w1v = zeros(3, 1);
else

if diff_mode == 0 % open
    if wm == 0 % no solution exists.
        w1m = (Mm - Mr - Ml) / (Jm + Jr + Jl);
        M = [-1, -1; ...
            (frac(2)*Jl), -(frac(1)*Jr)];
        n = [-2*w1m; frac(1)*Mr - frac(2)*Ml];
        w1v = [w1m; M \ n];
    else
        M = [2, -1, -1; ...
            (Jm*wm), (Jl*wl), (Jr*wr); ...
            0, (frac(2)*Jl), -(frac(1)*Jr)];
        n = [0; Mm*wm - Ml*wl - Mr*wr; frac(1)*Mr - frac(2)*Ml];
        w1v = M \ n;
    end
elseif diff_mode == 1 % closed
    if wm == 0
        w1m = (Mm - Mr - Ml) / (Jm + Jr + Jl);
        w1v = w1m*ones(3, 1);
    else
        M = [2, -1, -1; ...
            (Jm*wm), (Jl*wl), (Jr*wr); ...
            0, 1, -1];
        n = [0; Mm*wm - Ml*wl - Mr*wr; 0];
        w1v = M \ n;
    end
elseif (diff_mode > 0 && diff_mode < 1) % intermediate, "locking" differential
    w1v = (1 - diff_mode)*differential_function(w0v, Jm, Jl, Jr, Mm, Ml, Mr, 0, frac) + ...
        diff_mode*differential_function(w0v, Jm, Jl, Jr, Mm, Ml, Mr, 1, frac);
end


end
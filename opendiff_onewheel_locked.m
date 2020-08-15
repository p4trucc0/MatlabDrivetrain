function Ml = opendiff_onewheel_locked(wd, wu, Jd, Ju, Md, Mu)
% Find the torque to keep a locked wheel locked in an open diff.

if (Jd*wd + 2*Ju*wu) ~= 0

w1d = (Md * wd - Mu * wu) / (Jd*wd + 2*Ju*wu);
w1u = 2*w1d;

Ml = Mu + Ju*w1u;

else
    
end



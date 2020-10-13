function [Mr, yr, xr] = recompose_linear_system(M, y, x, i2switch)
% Recomposes linear system substituting known terms with unknowns

i_tot = [1:size(M, 1)];
i_inc = i_tot;
i_inc(i2switch) = [];
i_not = i2switch;

xi = x(i_inc);
xn = x(i_not);
yn = y(i_inc);
yi = y(i_not);

Mii = M(i_inc, i_inc);
Min = M(i_inc, i_not);
Mni = M(i_not, i_inc);
Mnn = M(i_not, i_not);

Mr = [Mii, zeros(length(i_inc), length(i_not)); ...
    Mni, -eye(length(i_not))];
xr = [xi; yi];

pMy = [eye(length(i_inc)), -Min; ...
    zeros(size(Mni)), -Mnn];
pyr = [yn; xn];
yr = pMy*pyr;




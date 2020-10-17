function [th2_v, th2d, M_v, Md, Mda, Mdp, add_param] = drivetrain_gen(diff_setup, th1_v, th1d, Mc, Mfv, Fxv, Jc, Jrv, rc, rd, t, ka, kp, Rv, brake_check)
% Patrucco, 2020
% General solution for all classes of free vs locked differential
% New approach based in fixed-dimension matrices as opposed to minimum
% number of linear independent conditions - less efficient but easier.
% TODO: handle the case of locking differential

% INPUT:
% diff_setup: integer describing diff setup configuration (1 - 12)
% th1_v: 4x1, angular speeds of the four tyres
% th1d: speed of the differential
% Mc: engine (or clutch) torque, minus sign if positive engine torque
% applied
% Mfv: braking torques at the wheels
% Fxv: longitudinal forces at wheel - road interface
% Jc: clutch (or engine + clutch) inertia
% Jrv: wheel inertia (4x1)
% rc: friction coefficient for clutch/(clutch + engine)
% rd: friction coefficient for differential
% t: transmission ratio
% ka: fraction of torque going to the front axle
% kp: fraction of torque going to the rear axle
% Rv: wheel radii
% brake_check: verify locked condition (still shafts)

% OUTPUT:
% th2_v: 4x1, angular accelerations of the four wheels
% th2d: angular acceleration of the differential
% M_v: torque applied to the four wheels
% Md: torque applied by the differential
% Mda: Front torque applied by the differential
% Mdp: Rear torque applied by the differential
% add_param: additional parameters

% Input parsing
diff_types = {'FWD free'; ... % 1
    'FWD locked'; ... % 2
    'RWD free'; ... % 3
    'RWD locked'; ... % 4
    '4WD locked central, front & rear free diffs'; ... % 5
    '4WD locked central, front free, rear locked'; ... % 6
    '4WD locked central, front locked, rear free'; ... % 7
    '4WD all-locked diffs'; ... % 8
    '4WD free central, front & rear free diffs'; ... % 9
    '4WD free central, front free, rear locked'; ... % 10
    '4WD free central, front locked, rear free'; ... % 11
    '4WD free central, front & rear locked'}; % 12

th1_as = th1_v(1); th1_ad = th1_v(2); th1_ps = th1_v(3); th1_pd = th1_v(4);
Mf_as = Mfv(1); Mf_ad = Mfv(2); Mf_ps = Mfv(3); Mf_pd = Mfv(4);
Fx_as = Fxv(1); Fx_ad = Fxv(2); Fx_ps = Fxv(3); Fx_pd = Fxv(4);
Jr_as = Jrv(1); Jr_ad = Jrv(2); Jr_ps = Jrv(3); Jr_pd = Jrv(4);
R_as = Rv(1); R_ad = Rv(2); R_ps = Rv(3); R_pd = Rv(4);

% main corpus
add_param = struct();
add_param.type_descr = diff_types{diff_setup};

% Main matrices
M_hi = [1, -1, -1, -1, -1, 0, 0, 0, 0, 0; ...
    1, 0, 0, 0, 0, Jc*(t^2), 0, 0, 0, 0; ...
    0, -1, 0, 0, 0, 0, Jr_as, 0, 0, 0; ...
    0, 0, -1, 0, 0, 0, 0, Jr_ad, 0, 0; ...
    0, 0, 0, -1, 0, 0, 0, 0, Jr_ps, 0; ...
    0, 0, 0, 0, -1, 0, 0, 0, 0, Jr_pd];
% known term, without brakes, upper part
u_sf_hi = [0; ...
    (-t*Mc -(rc*(t^2) + rd)*th1d); ...
    -Fx_as*R_as; ...
    -Fx_ad*R_ad; ...
    -Fx_ps*R_ps; ...
    -Fx_pd*R_pd];
u_f_hi = [0; ...
    0; ...
    -Mf_as; ...
    -Mf_ad; ...
    -Mf_ps; ...
    -Mf_pd];

switch(diff_setup)
    case 1 % FWD, free
        M_sub = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 1, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0, 2, -1, -1, 0, 0; ...
            1, -2, 0, 0, 0, 0, 0, 0, 0, 0];
        u_sub = zeros(4, 1);
    case 2 % FWD, locked
        M_sub = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 1, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0, 1, -1, 0, 0, 0; ...
            0, 0, 0, 0, 0, 1, 0, -1, 0, 0];
        u_sub = zeros(4, 1);
    case 3 % RWD, free
        M_sub = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0; ...
            0, 0, 1, 0, 0, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0, 2, 0, 0, -1, -1; ...
            1, 0, 0, -2, 0, 0, 0, 0, 0, 0];
        u_sub = zeros(4, 1);
    case 4 % RWD, locked 
        M_sub = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0; ...
            0, 0, 1, 0, 0, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0, 1, 0, 0, -1, 0; ...
            0, 0, 0, 0, 0, 1, 0, 0, 0, -1];
        u_sub = zeros(4, 1);
    case 5 % central locked diff, free front and rear
        M_sub = [0, 1, -1, 0, 0, 0, 0, 0, 0, 0; ...
            0, 0, 0, 1, -1, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0, 2, -1, -1, 0, 0; ...
            0, 0, 0, 0, 0, 2, 0, 0, -1, -1];
        u_sub = zeros(4, 1);
    case 6 % locked central diff, free front, locked rear
        M_sub = [0, 1, -1, 0, 0, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0, 1, 0, 0, -1, 0; ...
            0, 0, 0, 0, 0, 1, 0, 0, 0, -1; ...
            0, 0, 0, 0, 0, 2, -1, -1, 0, 0];
        u_sub = zeros(4, 1);
    case 7 % locked central diff, free rear, locked front
        M_sub = [0, 0, 0, -1, 1, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0, 1, -1, 0, 0, 0; ...
            0, 0, 0, 0, 0, 1, 0, -1, 0, 0; ...
            0, 0, 0, 0, 0, 2, 0, 0, -1, -1];
        u_sub = zeros(4, 1);
    case 8 % all diffs locked
        M_sub = [0, 0, 0, 0, 0, 1, -1, 0, 0, 0; ...
            0, 0, 0, 0, 0, 1, 0, -1, 0, 0; ...
            0, 0, 0, 0, 0, 1, 0, 0, -1, 0; ...
            0, 0, 0, 0, 0, 1, 0, 0, 0, -1];
        u_sub = zeros(4, 1);            
    case 9 % all diffs open
        M_sub = [0, 1, -1, 0, 0, 0, 0, 0, 0, 0; ...
            0, 0, 0, 1, -1, 0, 0, 0, 0, 0; ...
            ka, -1, -1, 0, 0, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0, 4, -1, -1, -1, -1];
        u_sub = zeros(4, 1);
    case 10 % C open, A open, P locked
        M_sub = [0, 1, -1, 0, 0, 0, 0, 0, 0, 0; ...
            ka, -1, -1, 0, 0, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0, 0, 0, 0, 1, -1; ...
            0, 0, 0, 0, 0, 4, -1, -1, -1, -1];
        u_sub = zeros(4, 1);
    case 11 % C open, A locked, P open
        M_sub = [0, 0, 0, 1, -1, 0, 0, 0, 0, 0; ...
            ka, -1, -1, 0, 0, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0, 0, 1, -1, 0, 0; ...
            0, 0, 0, 0, 0, 4, -1, -1, -1, -1];
        u_sub = zeros(4, 1);
    case 12 % C open, A and P locked
        M_sub = [ka, -1, -1, 0, 0, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0, 0, 1, -1, 0, 0; ...
            0, 0, 0, 0, 0, 0, 0, 0, 1, -1; ...
            0, 0, 0, 0, 0, 4, -1, -1, -1, -1];
        u_sub = zeros(4, 1);
end

M = [M_hi; M_sub];
u_sf = [u_sf_hi; u_sub];
u_f = [u_f_hi; u_sub];
u_cf = u_sf + u_f;
if brake_check
    x_sf = M \ u_sf;
    ind2switch = find(th1_v == 0) + 1; % find locked wheels
    
    % At least one shaft is locked, check if it remains so...
else
    x = M \ u_cf;
end

th2_v = x(7:10); %[th2_as; th2_ad; th2_ps; th2_pd];
th2d = x(6);
M_v = x(2:5);
Md = x(1);
Mda = M_v(1) + M_v(2);
Mdp = M_v(3) + M_v(4);



    function Mlock = find_locking_torques(i2s)
        [Mr, yr, xr] = recompose_linear_system(M, zeros(size(M, 1), 1), -x_sf, i2s);
        
        xx = Mr \ yr;
        Mlock = xx(length(xx)-length(i2s)+1:end);
    end







end
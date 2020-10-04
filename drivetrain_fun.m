function [th2_v, th2d, M_v, Md, Mda, Mdp, add_param] = drivetrain_fun(diff_setup, th1_v, th1d, Mc, Mfv, Fxv, Jc, Jrv, rc, rd, t, ka, kp, Rv)
% Patrucco, 2020
% General solution for all classes of free vs locked differential.
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

switch(diff_setup)
    case 1 % FWD, free
        [Md, th2_as, th2_ad, M_as, M_ad] = two_wd_free_diff(Fx_as, Fx_ad, th1_as, th1_ad, Jr_as, Jr_ad, R_as, R_ad, Mf_as, Mf_ad);
        th2_ps = disc_tyre_acc(J_ps, Mf_ps, Fx_ps, R_ps);
        th2_pd = disc_tyre_acc(J_pd, Mf_pd, Fx_pd, R_pd);
        Mda = Md;
        Mdp = 0;
        th2d = (th2_as + th2_ad)/2; % free diff relation
        M_ps = 0;
        M_pd = 0;
    case 2 % FWD, locked - MISSING FRICTION!
        J_red_f = (Jr_as/(t^2) + Jr_ad/(t^2) + Jc);
        M_red_f = -Mc - (1/t)*(Mf_as + Mf_ad + R_as*Fx_as + R_ad*Fx_ad);
        th2_f = M_red_f / J_red_f;
        th2d = t*th2_f;
        th2_as = t*th2_f;
        th2_ad = t*th2_f;
        th2_ps = disc_tyre_acc(J_ps, Mf_ps, Fx_ps, R_ps);
        th2_pd = disc_tyre_acc(J_pd, Mf_pd, Fx_pd, R_pd);
        Md = -t*Mc;
        Mda = Md;
        Mdp = 0;
        M_ps = 0;
        M_pd = 0;
        M_as = Mf_as + Fx_as*R_as + Jr_as*th2_as;
        M_ad = Mf_ad + Fx_ad*R_ad + Jr_ad*th2_ad;
    case 3 % RWD, free
        [Md, th2_ps, th2_pd, M_ps, M_pd] = two_wd_free_diff(Fx_ps, Fx_pd, th1_ps, th1_pd, Jr_ps, Jr_pd, R_ps, R_pd, Mf_ps, Mf_pd);
        th2_as = disc_tyre_acc(J_as, Mf_as, Fx_as, R_as);
        th2_ad = disc_tyre_acc(J_ad, Mf_ad, Fx_ad, R_ad);
        Mda = 0;
        Mdp = Md;
        th2d = (th2_ps + th2_pd)/2; % free diff relation
        M_as = 0;
        M_ad = 0;
    case 4 % RWD, locked - MISSING FRICTION!
        J_red_f = (Jr_ps/(t^2) + Jr_pd/(t^2) + Jc);
        M_red_f = -Mc - (1/t)*(Mf_ps + Mf_pd + R_ps*Fx_ps + R_pd*Fx_pd);
        th2_f = M_red_f / J_red_f;
        th2d = t*th2_f;
        th2_ps = t*th2_f;
        th2_pd = t*th2_f;
        th2_as = disc_tyre_acc(J_as, Mf_as, Fx_as, R_as);
        th2_ad = disc_tyre_acc(J_ad, Mf_ad, Fx_ad, R_ad);
        Md = -t*Mc;
        Mda = 0;
        Mdp = Md;
        M_as = 0;
        M_ad = 0;
        M_ps = Mf_ps + Fx_ps*R_ps + Jr_ps*th2_ps;
        M_pd = Mf_pd + Fx_pd*R_pd + Jr_pd*th2_pd;
    case 5 % central locked diff, free front and rear
        M_c5 = [1, 1, Jc*(t^2), 0, 0;
            -.5, 0, 2*Jr_as, -Jr_as, 0;
            -.5, 0, 0, Jr_ad, 0;
            0, -.5, 2*Jr_ps, 0, -Jr_ps;
            0, -.5, 0, Jr_ps, 0];
        U_c5 = [(-Mc*t - rc*(t^2)*th1d - rd*th1d); ...
            -Mf_as - Fx_as*R_as; ...
            -Mf_ad - Fx_ad*R_ad; ...
            -Mf_ps - Fx_ps*R_ps; ...
            -Mf_pd - Fx_pd*R_pd];
        x_c5 = M_c5 \ U_c5;
        Mda = x_c5(1);
        Mdp = x_c5(2);
        Md = Mda + Mdp;
        th2d = x_c5(3);
        th2_ad = x_c5(4);
        th2_pd = x_c5(5);
        th2_as = 2*th2d - th2_ad;
        th2_ps = 2*th2d - th2_pd;
        M_as = .5*Mda;
        M_ad = .5*Mda;
        M_ps = .5*Mdp;
        M_pd = .5*Mdp;
    case 6 % locked central diff, free front, locked rear
        M_c6 = [1, 1, Jc*(t^2), 0; ...
            0, -1, (Jr_ps + Jr_pd), 0; ...
            -.5, 0, 0, Jr_ad; ...
            -.5, 0, 2*Jr_as, -Jr_as];
        U_c6 = [(-Mc*t - rc*(t^2)*th1d - rd*th1d); ...
            -Mf_ps - Mf_pd - Fx_ps*R_ps - Fx_pd*R_pd; ...
            -Mf_ad - Fx_ad*R_ad; ...
            -Mf_as - Fx_as*R_as];
        x_c6 = M_c6 \ U_c6;
        Mda = x_c6(1);
        Mdp = x_c6(2);
        Md = Mda + Mdp;
        th2d = x_c6(3);
        th2_ad = x_c6(4);
        th2_as = 2*th2d - th2_ad;
        th2_ps = th2d;
        th2_pd = th2d;
        M_as = .5*Mda;
        M_ad = .5*Mda;
        M_ps = Mf_ps + Fx_ps*R_ps + Jr_ps*th2_ps;
        M_pd = Mf_pd + Fx_pd*R_pd + Jr_pd*th2_pd;
    case 7 % locked central diff, free rear, locked front
        M_c6 = [1, 1, Jc*(t^2), 0; ...
            -1, 0, (Jr_as + Jr_ad), 0; ...
            0, -.5, 0, Jr_pd; ...
            0, -.5, 2*Jr_ps, -Jr_ps];
        U_c6 = [(-Mc*t - rc*(t^2)*th1d - rd*th1d); ...
            -Mf_as - Mf_ad - Fx_as*R_as - Fx_ad*R_ad; ...
            -Mf_pd - Fx_pd*R_pd; ...
            -Mf_ps - Fx_ps*R_ps];
        x_c6 = M_c6 \ U_c6;
        Mda = x_c6(1);
        Mdp = x_c6(2);
        Md = Mda + Mdp;
        th2d = x_c6(3);
        th2_pd = x_c6(4);
        th2_ps = 2*th2d - th2_pd;
        th2_as = th2d;
        th2_ad = th2d;
        M_ps = .5*Mdp;
        M_pd = .5*Mdp;
        M_as = Mf_as + Fx_as*R_as + Jr_as*th2_as;
        M_ad = Mf_ad + Fx_ad*R_ad + Jr_ad*th2_ad;
    case 8 % all diffs locked
        J_red_d = (Jc*(t^2) + Jr_as + Jr_ad + Jr_ps + Jr_pd);
        M_red_d = -Mc*t - (rc*(t^2) + rd)*th1d - (Mf_as + Mf_ad + Mf_ps + ...
            Mf_pd) - (Fx_as*R_as + Fx_ad*R_ad + Fx_ps*R_ps + Rx_pd*R_pd);
        th2d = M_red_d / J_red_d;
        th2_as = th2d;
        th2_ad = th2d;
        th2_ps = th2d;
        th2_pd = th2d;
        Md = (Jr_as + Jr_ad + Jr_ps + Jr_pd)*th2d + (Mf_as + Mf_ad + Mf_ps + ...
            Mf_pd) + (Fx_as*R_as + Fx_ad*R_ad + Fx_ps*R_ps + Rx_pd*R_pd);
        M_as = Mf_as + Fx_as*R_as + Jr_as*th2_as;
        M_ad = Mf_ad + Fx_ad*R_ad + Jr_ad*th2_ad;
        M_ps = Mf_ps + Fx_ps*R_ps + Jr_ps*th2_ps;
        M_pd = Mf_pd + Fx_pd*R_pd + Jr_pd*th2_pd;
        Mda = M_as + M_ad;
        Mdp = M_ps + M_pd;
    case 9 % all diffs open
        M_c9 = [1, (Jc*(t^2)/4), (Jc*(t^2)/4), (Jc*(t^2)/4), (Jc*(t^2)/4); ...
            -.5*ka, Jr_as, 0, 0, 0; ...
            -.5*ka, 0, Jr_ad, 0, 0; ...
            -.5*kp, 0, 0, Jr_ps, 0; ...
            -.5*kp, 0, 0, 0, Jr_pd];
        U_c9 = [(-Mc*t - (rc*(t^2) + rd)*th1d); ...
            -Mf_as - Fx_as*R_as; ...
            -Mf_ad - Fx_ad*R_ad; ...
            -Mf_ps - Fx_ps*R_ps; ...
            -Mf_pd - Fx_pd*R_pd];
        x_c9 = M_c9 \ U_c9;
        Md = x_c9(1);
        Mda = Md*ka;
        Mdp = Md*kp;
        th2_as = x_c9(2);
        th2_ad = x_c9(3);
        th2_ps = x_c9(4);
        th2_pd = x_c9(5);
        th2d = .25*(th2_as + th2_ad + th2_ps + th2_pd);
        M_as = .5*ka*Md;
        M_ad = .5*ka*Md;
        M_ps = .5*kp*Md;
        M_pd = .5*kp*Md;
    case 10 % C open, A open, P locked
        M_ca = [1, (Jc*(t^2)/2), (Jc*(t^2)/4), (Jc*(t^2)/4); ...
            -kp, (Jr_ps + Jr_pd), 0, 0; ...
            -.5*ka, 0, Jr_as, 0; ...
            -.5*ka, 0, 0, Jr_ad];
        U_ca = [(-Mc*t - (rc*(t^2) + rd)*th1d); ...
            -(Mf_pd + Mf_ps + Fx_pd*R_pd + Fx_ps*R_ps); ...
            -Mf_as - Fx_as*R_as;
            -Mf_ad - Fx_ad*R_ad];
        x_ca = M_ca \ U_ca;
        Md = x_ca(1);
        th2dp = x_ca(2);
        th2_as = x_ca(3);
        th2_ad = x_ca(4);
        th2d = .25*(th2_as + th2_ad + 2*th2dp);
        th2_ps = th2dp;
        th2_pd = th2dp;
        Mda = ka*Md;
        Mdp = kp*Md;
        M_as = .5*Mda;
        M_ad = .5*Mda;
        M_ps = Mf_ps + Fx_ps*R_ps + Jr_ps*th2_ps;
        M_pd = Mf_pd + Fx_pd*R_pd + Jr_pd*th2_pd;
    case 11 % C open, A locked, P open
        M_cb = [1, (Jc*(t^2)/2), (Jc*(t^2)/4), (Jc*(t^2)/4); ...
            -ka, (Jr_as + Jr_ad), 0, 0; ...
            -.5*kp, 0, Jr_ps, 0; ...
            -.5*kp, 0, 0, Jr_pd];
        U_cb = [(-Mc*t - (rc*(t^2) + rd)*th1d); ...
            -(Mf_ad + Mf_as + Fx_ad*R_ad + Fx_as*R_as); ...
            -Mf_ps - Fx_ps*R_ps;
            -Mf_pd - Fx_pd*R_pd];
        x_cb = M_cb \ U_cb;
        Md = x_cb(1);
        th2da = x_cb(2);
        th2_ps = x_cb(3);
        th2_pd = x_cb(4);
        th2d = .25*(th2_ps + th2_pd + 2*th2da);
        th2_as = th2da;
        th2_ad = th2da;
        Mda = ka*Md;
        Mdp = kp*Md;
        M_ps = .5*Mdp;
        M_pd = .5*Mdp;
        M_as = Mf_as + Fx_as*R_as + Jr_as*th2_as;
        M_ad = Mf_ad + Fx_ad*R_ad + Jr_ad*th2_ad;
    case 12 % C open, A and P locked
        M_cc = [1, (Jc*(t^2)/2), (Jc*(t^2)/2); ...
            -ka, (J_as + J_ad), 0; ...
            -kp, 0, (J_ps + J_pd)];
        U_cc = [(-Mc*t - (rc*(t^2) + rd)*th1d); ...
             -(Mf_ad + Mf_as + Fx_ad*R_ad + Fx_as*R_as);
             -(Mf_pd + Mf_ps + Fx_pd*R_pd + Fx_ps*R_ps)];
        x_cc = M_cc \ U_cc;
        Md = x_cc(1);
        th2da = x_cc(2);
        th2dp = x_cc(3);
        th2d = .5*(th2da + th2dp);
        Mda = ka*Md;
        Mdp = kp*Md;
        th2_as = th2da;
        th2_ad = th2da;
        th2_ps = th2dp;
        th2_pd = th2dp;
        M_ps = Mf_ps + Fx_ps*R_ps + Jr_ps*th2_ps;
        M_pd = Mf_pd + Fx_pd*R_pd + Jr_pd*th2_pd;
        M_as = Mf_as + Fx_as*R_as + Jr_as*th2_as;
        M_ad = Mf_ad + Fx_ad*R_ad + Jr_ad*th2_ad;
end

th2_v = [th2_as; th2_ad; th2_ps; th2_pd];
M_v = [M_as; M_ad; M_ps; M_pd];



    function [Md, th2_s, th2_d, Mrs, Mrd] = two_wd_free_diff(Fxs, Fxd, th1_s, th1_d, Jrs, Jrd, Rs, Rd, Mfs, Mfd)
        M2wd = [1, Jc*((t^2)/2), Jc*((t^2)/2);
            -.5, Jrs, 0;
            -.5, 0, Jrd];
        u2wd = [(-t*Mc - (.5*rd + ((t^2)/2)*rc)*(th1_s + th1_d)); ...
            (-Fxs*Rs - Mfs);
            (-Fxd*Rd - Mfd)];
        x2wd = M2wd \ u2wd;
        Md = x2wd(1);
        th2_s = x2wd(2);
        th2_d = x2wd(3);
        Mrs = -.5*Mc;
        Mrd = -.5*Mc;
    end

    function th2_free = disc_tyre_acc(Jt, Mft, Fxt, Rt)
        th2_free = (-Mft - Fxt*Rt) / Jt;
    end


end
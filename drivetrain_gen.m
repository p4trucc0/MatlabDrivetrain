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


CHECK_DIFF_SPEED = true; % Manually correct input diff speed to account for integration quirks

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
        M_sub = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 1, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0, 2, -1, -1, 0, 0; ...
            1, -2, 0, 0, 0, 0, 0, 0, 0, 0];
        u_sub = zeros(4, 1);
        if CHECK_DIFF_SPEED
            th1d = .5*(th1_ad + th1_as);
        end
    case 2 % FWD, locked
        M_sub = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 1, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0, 1, -1, 0, 0, 0; ...
            0, 0, 0, 0, 0, 1, 0, -1, 0, 0];
        u_sub = zeros(4, 1);
        if CHECK_DIFF_SPEED
            th1d = th1_ad;
        end
    case 3 % RWD, free
        M_sub = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0; ...
            0, 0, 1, 0, 0, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0, 2, 0, 0, -1, -1; ...
            1, 0, 0, -2, 0, 0, 0, 0, 0, 0];
        u_sub = zeros(4, 1);
        if CHECK_DIFF_SPEED
            th1d = .5*(th1_pd + th1_ps);
        end
    case 4 % RWD, locked 
        M_sub = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0; ...
            0, 0, 1, 0, 0, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0, 1, 0, 0, -1, 0; ...
            0, 0, 0, 0, 0, 1, 0, 0, 0, -1];
        u_sub = zeros(4, 1);
        if CHECK_DIFF_SPEED
            th1d = th1_pd;
        end
    case 5 % central locked diff, free front and rear
        M_sub = [0, 1, -1, 0, 0, 0, 0, 0, 0, 0; ...
            0, 0, 0, 1, -1, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0, 2, -1, -1, 0, 0; ...
            0, 0, 0, 0, 0, 2, 0, 0, -1, -1];
        u_sub = zeros(4, 1);
        if CHECK_DIFF_SPEED
            th1d = .5*(th1_pd + th1_ps);
        end
    case 6 % locked central diff, free front, locked rear
        M_sub = [0, 1, -1, 0, 0, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0, 1, 0, 0, -1, 0; ...
            0, 0, 0, 0, 0, 1, 0, 0, 0, -1; ...
            0, 0, 0, 0, 0, 2, -1, -1, 0, 0];
        u_sub = zeros(4, 1);
        if CHECK_DIFF_SPEED
            th1d = .5*(th1_pd + th1_ps);
        end
    case 7 % locked central diff, free rear, locked front
        M_sub = [0, 0, 0, -1, 1, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0, 1, -1, 0, 0, 0; ...
            0, 0, 0, 0, 0, 1, 0, -1, 0, 0; ...
            0, 0, 0, 0, 0, 2, 0, 0, -1, -1];
        u_sub = zeros(4, 1);
        if CHECK_DIFF_SPEED
            th1d = .5*(th1_pd + th1_ps);
        end
    case 8 % all diffs locked
        M_sub = [0, 0, 0, 0, 0, 1, -1, 0, 0, 0; ...
            0, 0, 0, 0, 0, 1, 0, -1, 0, 0; ...
            0, 0, 0, 0, 0, 1, 0, 0, -1, 0; ...
            0, 0, 0, 0, 0, 1, 0, 0, 0, -1];
        u_sub = zeros(4, 1);
        if CHECK_DIFF_SPEED
            th1d = th1_as;
        end            
    case 9 % all diffs open
        M_sub = [0, 1, -1, 0, 0, 0, 0, 0, 0, 0; ...
            0, 0, 0, 1, -1, 0, 0, 0, 0, 0; ...
            ka, -1, -1, 0, 0, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0, 4, -1, -1, -1, -1];
        u_sub = zeros(4, 1);
        if CHECK_DIFF_SPEED
            th1d = .25*(th1_as + th1_ad + th1_ps + th1_pd);
        end        
    case 10 % C open, A open, P locked
        M_sub = [0, 1, -1, 0, 0, 0, 0, 0, 0, 0; ...
            ka, -1, -1, 0, 0, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0, 0, 0, 0, 1, -1; ...
            0, 0, 0, 0, 0, 4, -1, -1, -1, -1];
        u_sub = zeros(4, 1);
        if CHECK_DIFF_SPEED
            th1d = .25*(th1_as + th1_ad + th1_ps + th1_pd);
        end        
    case 11 % C open, A locked, P open
        M_sub = [0, 0, 0, 1, -1, 0, 0, 0, 0, 0; ...
            ka, -1, -1, 0, 0, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0, 0, 1, -1, 0, 0; ...
            0, 0, 0, 0, 0, 4, -1, -1, -1, -1];
        u_sub = zeros(4, 1);
        if CHECK_DIFF_SPEED
            th1d = .25*(th1_as + th1_ad + th1_ps + th1_pd);
        end        
    case 12 % C open, A and P locked
        M_sub = [ka, -1, -1, 0, 0, 0, 0, 0, 0, 0; ...
            0, 0, 0, 0, 0, 0, 1, -1, 0, 0; ...
            0, 0, 0, 0, 0, 0, 0, 0, 1, -1; ...
            0, 0, 0, 0, 0, 4, -1, -1, -1, -1];
        u_sub = zeros(4, 1);
        if CHECK_DIFF_SPEED
            th1d = .25*(th1_as + th1_ad + th1_ps + th1_pd);
        end        
end

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

M = [M_hi; M_sub];
u_f = [u_f_hi; zeros(4, 1)];
if brake_check
    tol_brk = 0.01; % max speed.
    disc_status = abs(th1_v) < tol_brk;
    lock_status = ~disc_status; % the unstopped ones are not to be investigated.
    keep_checking = true;
    smm = [zeros(2, 4); eye(4)];
    M_ba = [M_hi, smm; ...
        zeros(4, 6), eye(4), zeros(4, 4); ...
        get_lock_subm()];
    M_ba_c = M_ba; % current
    % U_ba_c = U_ba; % current
    u_sf = [u_sf_hi; zeros(4, 1)];
    x_sf = M \ u_sf;
    %Mfc = abs(Mfv).*-sign(x_sf(7:10));
    Mfc = Mfv;
    Mfc(disc_status) = Mfc(disc_status)*2;
    checkno = 0;
    while keep_checking
        % Introduce known braking effect!
        if (~all(lock_status))
        rows2excl = 2 + find(lock_status);
        M_ba = [M_hi, smm; ...
        zeros(4, 6), eye(4), zeros(4, 4); ...
        get_lock_subm()];
        M_ba_c = M_ba; % current
        M_ba_c(4+rows2excl, :) = generate_lock_identities(lock_status); 
        u_sf_hi_c = u_sf_hi;
        u_sf_hi_c(rows2excl) = u_sf_hi_c(rows2excl) - Mfc(lock_status);
        u_sf = [u_sf_hi_c; zeros(4, 1)];
        x_sf = M \ u_sf;
        U_ba_c = [zeros(6, 1); -x_sf(7:10);  zeros(4, 1)];
        U_ba_c(4+rows2excl) = 0; %Mfc(lock_status);
        %if abs(det(M_ba_c)) > tol_brk
        xf_p = M_ba_c \ U_ba_c;
        %end % else: take previous values as good. They should not change (intuition)...
        Mfp_c = xf_p(11:14);
        brk_trq_relation = abs(Mfp_c) <= abs(Mfc);
        end
        if all(lock_status | brk_trq_relation) %(all(lock_status) || all(brk_trq_relation))
            keep_checking = false;
            Mfp_c(lock_status) = Mfc(lock_status);
            Mfe = Mfp_c;
        else
            lock_status(~brk_trq_relation) = true; % greater than limit braking torque - Not blocked
        end
        checkno = checkno + 1;
        %keyboard
    end
    % END WHILE
    u_f = [0; ...
    0; ...
    -Mfe(1); ...
    -Mfe(2); ...
    -Mfe(3); ...
    -Mfe(4); 
    zeros(4, 1)];
end
u_sf = [u_sf_hi; u_sub];
u_cf = u_sf + u_f;
add_param.Mfv = -u_f(3:6); % braking forces
add_param.th1d = th1d;
x = M \ u_cf;
% end

th2_v = x(7:10); %[th2_as; th2_ad; th2_ps; th2_pd];
th2d = x(6);
M_v = x(2:5);
Md = x(1);
Mda = M_v(1) + M_v(2);
Mdp = M_v(3) + M_v(4);



    function subMl_known = generate_lock_identities(ls)
        subMl_nw = [zeros(4, 10), eye(4)];
        ind_known = find(ls);
        subMl_known = subMl_nw(ind_known, :);
    end

    % Returns submatrix for brake-unlock conditions
    % TODO: For locked diffs, consider case of unequal braking torque
    % left/right
    function subMl = get_lock_subm()
        bap = (Mfv(1) + Mfv(2)) / (Mfv(3) + Mfv(4)); % brake ratio: front / rear
        switch(diff_setup)
            case 1 % FWD, free
                subMl = [zeros(1, 5), 1, -.5, -.5, 0, 0, zeros(1, 4); ...
                    0, 1, -1, zeros(1, 11); ...
                    0, 0, 0, 1, 0, zeros(1, 9); ...
                    0, 0, 0, 0, 1, zeros(1, 9)]; % front torque repart.
            case 2 % FWD, locked
                subMl = [zeros(1, 5), 1, -1, 0, 0, 0, zeros(1, 4); ...
                    zeros(1, 10), 1, -1, 0, 0; ...
                    0, 0, 0, 1, 0, zeros(1, 9); ...
                    0, 0, 0, 0, 1, zeros(1, 9)]; % front braking forces equal; no rear applied torque.
                if any(lock_status(1:2))
                    subMl(2,:) = [zeros(1, 5), 0, 1, -1, 0, 0, zeros(1, 4)];
                end
            case 3 % RWD, free
                subMl = [zeros(1, 5), 1, 0, 0, -.5, -.5, zeros(1, 4); ...
                    0, 0, 0, 1, -1, zeros(1, 9); ...
                    0, 1, 0, 0, 0, zeros(1, 9); ...
                    0, 0, 1, 0, 0, zeros(1, 9)]; % rear torque repart.
            case 4 % RWD, locked
                subMl = [zeros(1, 5), 1, 0, 0, 0, -1, zeros(1, 4); ...
                    zeros(1, 10), 0, 0, -1, 1; ...
                    0, 1, 0, 0, 0, zeros(1, 9); ...
                    0, 0, 1, 0, 0, zeros(1, 9)]; % rear braking forces equal; no front applied torque.
                if any(lock_status(3:4))
                    subMl(2,:) = [zeros(1, 5), 0, 0, 0, 1, -1, zeros(1, 4)];
                end
            case 5 % central locked diff, free front and rear
                subMl = [zeros(1, 5), 1, -.5, -.5, 0, 0, zeros(1, 4); ...
                    0, 1, -1, 0, 0, zeros(1, 9); ...
                    0, 0, 0, 1, -1, zeros(1, 9); ...
                    zeros(1, 10), -1, -1, bap, bap]; % impose correct brake force distr.
            case 6 % locked central diff, free front, locked rear
                subMl = [zeros(1, 5), 1, -.5, -.5, 0, 0, zeros(1, 4); ...
                    0, 1, -1, 0, 0, zeros(1, 9); ...
                    zeros(1, 10), 0, 0, 1, -1; ...
                    zeros(1, 10), -1, -1, bap, bap]; % impose correct brake force distr and equal braking rear axle.
                if any(lock_status(3:4))
                    subMl(3,:) = [zeros(1, 5), 0, 0, 0, 1, -1, zeros(1, 4)];
                end
            case 7 % locked central diff, free rear, locked front
                subMl = [zeros(1, 5), 1, -1, 0, 0, 0, zeros(1, 4); ...
                    0, 0, 0, 1, -1, zeros(1, 9); ...
                    zeros(1, 10), 1, -1, 0, 0; ...
                    zeros(1, 10), -1, -1, bap, bap]; % impose correct brake force distr and equal braking front axle.
                if any(lock_status(1:2))
                    subMl(3,:) = [zeros(1, 5), 0, 1, -1, 0, 0, zeros(1, 4)];
                end
            case 8 % all diffs locked
                subMl = [zeros(1, 5), 1, -1, 0, 0, 0, zeros(1, 4); ...
                    zeros(1, 10), 0, 0, 1, -1; ...
                    zeros(1, 10), 1, -1, 0, 0; ...
                    zeros(1, 10), -1, -1, bap, bap]; % brake forces should respect static balancement.
                if any(lock_status(1:2))
                    subMl(3,:) = [zeros(1, 5), 0, 1, -1, 0, 0, zeros(1, 4)];
                end
                if any(lock_status(3:4))
                    subMl(2,:) = [zeros(1, 5), 0, 0, 0, 1, -1, zeros(1, 4)];
                end
            case 9 % all diffs open
                subMl = [zeros(1, 5), 1, -.25, -.25, -.25, -.25, zeros(1, 4); ...
                    0, 1, -1, 0, 0, zeros(1, 9); ...
                    0, 0, 0, 1, -1, zeros(1, 9); ...
                    ka, -1, -1, 0, 0, zeros(1, 9)]; % impose front torque distr.
            case 10 % C open, A open, P locked
                subMl = [zeros(1, 5), 1, -.25, -.25, -.5, 0, zeros(1, 4); ...
                    0, 1, -1, 0, 0, zeros(1, 9); ...
                    zeros(1, 10), 0, 0, 1, -1; ...
                    ka, -1, -1, 0, 0, zeros(1, 9)]; 
                if any(lock_status(3:4))
                    subMl(3,:) = [zeros(1, 5), 0, 0, 0, 1, -1, zeros(1, 4)];
                end
            case 11 % C open, A locked, P open
                subMl = [zeros(1, 5), 1, -.5, 0, -.25, -.25, zeros(1, 4); ...
                    0, 0, 0, 1, -1, zeros(1, 9); ...
                    zeros(1, 10), 1, -1, 0, 0; ...
                    ka, -1, -1, 0, 0, zeros(1, 9)]; 
                if any(lock_status(1:2))
                    subMl(3,:) = [zeros(1, 5), 0, 1, -1, 0, 0, zeros(1, 4)];
                end
            case 12 % C open, A and P locked
                subMl = [zeros(1, 5), 1, -.5, 0, -.5, 0, zeros(1, 4); ...
                    zeros(1, 10), 0, 0, 1, -1; ...
                    zeros(1, 10), 1, -1, 0, 0; ...
                    ka, -1, -1, 0, 0, zeros(1, 9)]; 
                if any(lock_status(1:2))
                    subMl(3,:) = [zeros(1, 5), 0, 1, -1, 0, 0, zeros(1, 4)];
                end
                if any(lock_status(3:4))
                    subMl(2,:) = [zeros(1, 5), 0, 0, 0, 1, -1, zeros(1, 4)];
                end
        end
    end






end
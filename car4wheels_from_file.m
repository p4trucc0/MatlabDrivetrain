function car = car4wheels_from_file(file_in)
% Patrucco, 2021
% Generate a car object from an ascii descriptive file.

vp = veh_param_from_ascii(file_in);

m = Engine(vp.engine_lambda_v, vp.engine_rpm_v, vp.engine_torque_map, vp.engine_fv_0, vp.engine_fv_1, vp.engine_Jm);
f = Clutch(vp.clutch_kf, vp.clutch_J, vp.clutch_r, vp.clutch_ef_thr, vp.clutch_sp_thr);
c = Gearbox(vp.gearbox_ratios, vp.gearbox_efficiencies, vp.gearbox_has_reverse);
d = Differential2Axle_v2(vp.differential_final_drive, [vp.differential_td_a, ...
    1-vp.differential_td_a], vp.differential_diff_type, vp.differential_r);
b_f = Brake(vp.brakef_f, vp.brakef_R1, vp.brakef_R2, vp.brakef_alpha, vp.brakef_max_Q);
b_r = Brake(vp.braker_f, vp.braker_R1, vp.braker_R2, vp.braker_alpha, vp.braker_max_Q);
cnt = Controls();

nbd = struct();
nbd.x = vp.brakedistr_x;
nbd.y_rf = vp.brakedistr_y;

bs = BrakeSystem2Axle(b_f, b_r, cnt, nbd);

ptf = parse_tyre_params(vp.wheelf_pacejka_std_file);
ptr = parse_tyre_params(vp.wheelr_pacejka_std_file);

w_fl = Wheel(vp.wheelf_R, vp.wheelf_J, ptf);
w_fr = Wheel(vp.wheelf_R, vp.wheelf_J, ptf);
w_rl = Wheel(vp.wheelr_R, vp.wheelr_J, ptr);
w_rr = Wheel(vp.wheelr_R, vp.wheelr_J, ptr);

drt = Drivetrain2Axle_v2(m, f, c, d, bs, cnt, w_fl, w_rl);

vg = struct(); % vehicle geometry.
vg.Clf = 0.0; vg.Clr = 0.0; vg.Cx = vp.body_drag_coeff;
vg.d_fl = vp.susp_f_d; vg.d_fr = -vp.susp_f_d;
vg.d_rl = vp.susp_r_d; vg.d_rr = -vp.susp_r_d;
vg.h_fl = vp.susp_f_h; vg.h_fr = vp.susp_f_h;
vg.h_rl = vp.susp_r_h; vg.h_rr = vp.susp_r_h;
vg.Jx = vp.body_Jx; vg.Jy = vp.body_Jy; vg.Jz = vp.body_Jz;
vg.kp_fl = vp.susp_f_kp; vg.kp_fr = vp.susp_f_kp; vg.kp_rl = vp.susp_r_kp; vg.kp_rr = vp.susp_r_kp;
vg.ks_fl = vp.susp_f_ks; vg.ks_fr = vp.susp_f_ks; vg.ks_rl = vp.susp_r_ks; vg.ks_rr = vp.susp_r_ks;
vg.l_fl_ind = vp.susp_f_l_ind; vg.l_fr_ind = vp.susp_f_l_ind; vg.l_rl_ind = vp.susp_r_l_ind; vg.l_rr_ind = vp.susp_r_l_ind;
vg.mc = vp.body_mass;
vg.p_fl = vp.susp_f_p; vg.p_fr = vp.susp_f_p;
vg.p_rl = -vp.susp_r_p; vg.p_rr = -vp.susp_r_p;
vg.rp_fl = vp.susp_f_rp; vg.rp_fr = vp.susp_f_rp; vg.rp_rl = vp.susp_r_rp; vg.rp_rr = vp.susp_r_rp;
vg.rs_fl = vp.susp_f_rs; vg.rs_fr = vp.susp_f_rs; vg.rs_rl = vp.susp_r_rs; vg.rs_rr = vp.susp_r_rs;
vg.r_fl_ind = vp.susp_f_r_ind; vg.r_fr_ind = vp.susp_f_r_ind; vg.r_rl_ind = vp.susp_r_r_ind; vg.r_rr_ind = vp.susp_r_r_ind;
vg.S = vp.body_front_surface;
vg.s_fl = vp.susp_f_s; vg.s_fr = -vp.susp_f_s;
vg.s_rl = vp.susp_r_s; vg.s_rr = -vp.susp_r_s;
vg.xb = vp.body_xb; vg.yb = vp.body_yb; vg.zb = vp.body_zb;

car = CarWithFourWheels(vg, w_fl, w_fr, w_rl, w_rr, drt, cnt, vp.steering_scalar_sensitivity);


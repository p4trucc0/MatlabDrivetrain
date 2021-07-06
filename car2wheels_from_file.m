function car = car2wheels_from_file(file_in)
% Patrucco, 2021
% Generates a "CarWithTwoWheels" object from the standard ascii format

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

w_f = Wheel(vp.wheelf_R, vp.wheelf_J, ptf);
w_r = Wheel(vp.wheelr_R, vp.wheelr_J, ptr);

drt = Drivetrain2Axle_v2(m, f, c, d, bs, cnt, w_f, w_r);

[x_a, x_p] = axledist_from_wbase(vp.body_wheelbase, vp.body_front_weight_distr);

car = CarWithTwoWheels(vp.body_mass, ...
                w_f, w_r, ...
                vp.body_front_surface, vp.body_drag_coeff, drt, ...
                x_a, x_p, vp.body_z_cog, vp.body_lift_coeff);
classdef CarWithFourWheels < handle
    
    % Patrucco, 2021
    % Port of C++ version of the same object
    
    properties
        vg = []; % vehicle geometry struct
        wheel_fl = [];
        wheel_fr = [];
        wheel_rl = [];
        wheel_rr = []; % wheel objects.
        drivetrain = [];
        controls = [];
        steering_sensitivity = [];
        min_speed_for_slip = 1.0;
        tol_speed_static = 0.01;
        apply_tyre_forces = true;
    end
    
    methods
        
        % Constructor
        function obj = CarWithFourWheels(n_vg, n_wheel_fl, n_wheel_fr, ...
                n_wheel_rl, n_wheel_rr, n_drivetrain, n_controls, ...
                n_steering_sensitivity)
            obj.vg = n_vg;
            obj.wheel_fl = n_wheel_fl;
            obj.wheel_fr = n_wheel_fr;
            obj.wheel_rl = n_wheel_rl;
            obj.wheel_rr = n_wheel_rr;
            obj.drivetrain = n_drivetrain;
            obj.controls = n_controls;
            obj.steering_sensitivity = n_steering_sensitivity;
        end
        
        function kk = eval_long_slip(obj, w, r, v)
            if (abs(v) < obj.min_speed_for_slip)
                kk = (100.0 * (w*r - v)) / obj.min_speed_for_slip;
            else
                kk = (100.0 * (w*r - v)) / abs(v) ;
            end
        end
        
        function out = eval_lat_slip(obj, ang_wheel, vx, vy)
            SLIPANGLE_ZEROTOL = 0.01;
            if ((abs(vx) < SLIPANGLE_ZEROTOL) && (abs(vy) < SLIPANGLE_ZEROTOL))
                out = 0.0;
            else
                if (abs(vx) < SLIPANGLE_ZEROTOL)
                    if (vy > 0.0)
                        atyx = pi/2.0;
                    else
                        atyx = -pi/2.0;
                    end
                else
                    if (vx > 0.0)
                        atyx = atan(vy/vx);
                    else
                        atyx = -atan(vy/vx);
                    end
                end
                out = (ang_wheel - atyx);
            end
        end
        
        % x0v: positions
        % x1v: speeds
        % x2v_p: accelerations in previous time frame
        % x2v: accelerations
        % dap: additional parameters.
        function [x2v, par_out] = get_acc(obj, x0v, x1v, x2v_p)
            % Copy kinetic quantities
            xc_1 = x1v(7);
            yc_1 = x1v(8);
            zc_1 = x1v(9);
            rho_1 = x1v(10);
            beta_1 = x1v(11);
            sigma_1 = x1v(12);
            l_fl_1 = x1v(13);
            l_fr_1 = x1v(14);
            l_rl_1 = x1v(15);
            l_rr_1 = x1v(16);
            theta_fl_1 = x1v(3);
            theta_fr_1 = x1v(4);
            theta_rl_1 = x1v(5);
            theta_rr_1 = x1v(6);
            theta_m_1 = x1v(1); % engine speed
            theta_c_1 = x1v(2); % clutch speed
            xc_0 = x0v(7);
            yc_0 = x0v(8);
            zc_0 = x0v(9);
            rho_0 = x0v(10);
            beta_0 = x0v(11);
            sigma_0 = x0v(12);
            l_fl_0 = x0v(13);
            l_fr_0 = x0v(14);
            l_rl_0 = x0v(15);
            l_rr_0 = x0v(16);
            theta_fl_0 = x0v(3);
            theta_fr_0 = x0v(4);
            theta_rl_0 = x0v(5);
            theta_rr_0 = x0v(6);
            theta_m_0 = x0v(1); % engine pos
            theta_c_0 = x0v(2); % clutch pos
            
            q_0 = x0v(1:10);
            q_1 = x1v(1:10);
            
            % Terrain speed (unused)
            zpn_fl_0 = 0;
            zpn_fl_1 = 0;
            zpn_fr_0 = 0;
            zpn_fr_1 = 0;
            zpn_rl_0 = 0;
            zpn_rl_1 = 0;
            zpn_rr_0 = 0;
            zpn_rr_1 = 0;
            
            % Steering (TODO: Improve this... don't you know Ackerman?)
            phi_fl = obj.controls.ste_wheel*obj.steering_sensitivity;
            phi_fr = obj.controls.ste_wheel*obj.steering_sensitivity;
            phi_rl = 0;
            phi_rr = 0;
            
            
            % Physical parameters of the model.
            % car body center of gravity position.
            mc = obj.vg.mc; % kg.
            Ixc = obj.vg.Jx; %kg * m
            Iyc = obj.vg.Jy; %kg * m
            Izc = obj.vg.Jz; %kg * m
            xb = obj.vg.xb; %m, distance of COG from car-centered RS in car RS.
            yb = obj.vg.yb; %m
            zb = obj.vg.zb; %m
            
            % Wheel data.
            mr_fl = obj.wheel_fl.m; mr_fr = obj.wheel_fr.m; mr_rl = obj.wheel_rl.m; mr_rr = obj.wheel_rr.m;
            Jr_fl = obj.wheel_fl.J; Jr_fr = obj.wheel_fr.J; Jr_rl = obj.wheel_rl.J; Jr_rr = obj.wheel_rr.J;
            rr_fl = obj.wheel_fl.R; rr_fr = obj.wheel_fr.R; rr_rl = obj.wheel_rl.R; rr_rr = obj.wheel_rr.R;
            
            % Suspension
            p_fl = obj.vg.p_fl; p_fr = obj.vg.p_fr; p_rl = obj.vg.p_rl; p_rr = obj.vg.p_rr;
            s_fl = obj.vg.s_fl; s_fr = obj.vg.s_fr; s_rl = obj.vg.s_rl; s_rr = obj.vg.s_rr;
            d_fl = obj.vg.d_fl; d_fr = obj.vg.d_fr; d_rl = obj.vg.d_rl; d_rr = obj.vg.d_rr;
            h_fl = obj.vg.h_fl; h_fr = obj.vg.h_fr; h_rl = obj.vg.h_rl; h_rr = obj.vg.h_rr;
            l_fl_ind = obj.vg.l_fl_ind; l_fr_ind = obj.vg.l_fr_ind; l_rl_ind = obj.vg.l_rl_ind; l_rr_ind = obj.vg.l_rr_ind;
            r_fl_ind = obj.vg.r_fl_ind; r_fr_ind = obj.vg.r_fr_ind; r_rl_ind = obj.vg.r_rl_ind; r_rr_ind = obj.vg.r_rr_ind;
            ks_fl = obj.vg.ks_fl; ks_fr = obj.vg.ks_fr; ks_rl = obj.vg.ks_rl; ks_rr = obj.vg.ks_rr;
            kp_fl = obj.vg.kp_fl; kp_fr = obj.vg.kp_fr; kp_rl = obj.vg.kp_rl; kp_rr = obj.vg.kp_rr;
            rs_fl = obj.vg.rs_fl; rs_fr = obj.vg.rs_fr; rs_rl = obj.vg.rs_rl; rs_rr = obj.vg.rs_rr;
            rp_fl = obj.vg.rp_fl; rp_fr = obj.vg.rp_fr; rp_rl = obj.vg.rp_rl; rp_rr = obj.vg.rp_rr;
            
            tyre_param_fl = obj.wheel_fl.pacejka;
            tyre_param_fr = obj.wheel_fr.pacejka;
            tyre_param_rl = obj.wheel_rl.pacejka;
            tyre_param_rr = obj.wheel_rr.pacejka;
            
            %% Start of Model-related equations.
            %% Mass matrices and terms.
            % Car body, from "3/5/20 - 7"
            L_c_0 = generate_jacobian(rho_0, beta_0, sigma_0);
            X_BO_c = v2M([xb; yb; zb])'; % transposed to see whether it corrects roll problem.
            A_c = generate_a(rho_0, beta_0, sigma_0);
            Lmc_12 = L_c_0 * X_BO_c * A_c;
            L_c_qc = [eye(3)       Lmc_12;
                zeros(3)       A_c];
            L_c_qt = [L_c_qc       zeros(6, 4)];
            M_car_phys = diag([mc mc mc Ixc Iyc Izc]);
            M_c = L_c_qt'*M_car_phys*L_c_qt; % in total coordinates.
            
            % Wheels, as a point mass. From "2/5/20 - 4"
            % generate_wheel_jacobian(rho, beta, sigma, p, s, d, h, l)
            L_r_fl_qc = generate_wheel_jacobian(rho_0, beta_0, sigma_0, p_fl, s_fl, d_fl, h_fl, l_fl_0);
            L_r_fr_qc = generate_wheel_jacobian(rho_0, beta_0, sigma_0, p_fr, s_fr, d_fr, h_fr, l_fr_0);
            L_r_rl_qc = generate_wheel_jacobian(rho_0, beta_0, sigma_0, p_rl, s_rl, d_rl, h_rl, l_rl_0);
            L_r_rr_qc = generate_wheel_jacobian(rho_0, beta_0, sigma_0, p_rr, s_rr, d_rr, h_rr, l_rr_0);
            
            L_r_fl_qt = [L_r_fl_qc(:, 1:6) zeros(3,0) L_r_fl_qc(:, 7) zeros(3,3)];
            L_r_fr_qt = [L_r_fr_qc(:, 1:6) zeros(3,1) L_r_fr_qc(:, 7) zeros(3,2)];
            L_r_rl_qt = [L_r_rl_qc(:, 1:6) zeros(3,2) L_r_rl_qc(:, 7) zeros(3,1)];
            L_r_rr_qt = [L_r_rr_qc(:, 1:6) zeros(3,3) L_r_rr_qc(:, 7) zeros(3,0)];
            
            M_r_fl_phys = mr_fl*eye(3);
            M_r_fr_phys = mr_fr*eye(3);
            M_r_rl_phys = mr_rl*eye(3);
            M_r_rr_phys = mr_rr*eye(3);
            
            M_r_fl = L_r_fl_qt'*M_r_fl_phys*L_r_fl_qt;
            M_r_fr = L_r_fr_qt'*M_r_fr_phys*L_r_fr_qt;
            M_r_rl = L_r_rl_qt'*M_r_rl_phys*L_r_rl_qt;
            M_r_rr = L_r_rr_qt'*M_r_rr_phys*L_r_rr_qt;
            
            % Total Mass Matrix
            M = M_c + M_r_fl + M_r_fr + M_r_rl + M_r_rr;
            
            % Derivative of Mass Matrix (approximated form)
            % from "8/5/20 - 5"
                       
            % Alternative way to calculate this.
            M1_c_q6_2 = generate_approx_M_derivative_m2(M_car_phys, rho_0, beta_0, sigma_0, rho_1, beta_1, sigma_1, xb, yb, zb, 0, 0, 0);
            M1_r_fl_q6_2 = generate_approx_M_derivative_m2([M_r_fl_phys zeros(3); zeros(3, 6)], rho_0, beta_0, sigma_0, rho_1, beta_1, sigma_1, p_fl, (s_fl + d_fl), (h_fl - l_fl_0), 0, 0, -l_fl_1);
            M1_r_fr_q6_2 = generate_approx_M_derivative_m2([M_r_fr_phys zeros(3); zeros(3, 6)], rho_0, beta_0, sigma_0, rho_1, beta_1, sigma_1, p_fr, (s_fr + d_fr), (h_fr - l_fr_0), 0, 0, -l_fr_1);
            M1_r_rl_q6_2 = generate_approx_M_derivative_m2([M_r_rl_phys zeros(3); zeros(3, 6)], rho_0, beta_0, sigma_0, rho_1, beta_1, sigma_1, p_rl, (s_rl + d_rl), (h_rl - l_rl_0), 0, 0, -l_rl_1);
            M1_r_rr_q6_2 = generate_approx_M_derivative_m2([M_r_rr_phys zeros(3); zeros(3, 6)], rho_0, beta_0, sigma_0, rho_1, beta_1, sigma_1, p_rr, (s_rr + d_rr), (h_rr - l_rr_0), 0, 0, -l_rr_1);
            % Assembling:
            M1_q6 = M1_c_q6_2(1:6, 1:6) + M1_r_fl_q6_2(1:6, 1:6) + M1_r_fr_q6_2(1:6, 1:6) + ...
                M1_r_rl_q6_2(1:6, 1:6) + M1_r_rr_q6_2(1:6, 1:6);
            M1 = [M1_q6 M1_r_fl_q6_2(1:6, 7) M1_r_fr_q6_2(1:6, 7) M1_r_rl_q6_2(1:6, 7) M1_r_rr_q6_2(1:6, 7);
                M1_r_fl_q6_2(7, 1:6) zeros(1, 4);
                M1_r_fr_q6_2(7, 1:6) zeros(1, 4);
                M1_r_rl_q6_2(7, 1:6) zeros(1, 4);
                M1_r_rr_q6_2(7, 1:6) zeros(1, 4)];
            
            
            % Derivative of Kinetic Energy with respect to Position
            % (Approximated form)
            % from "8/5/20 - 4"
            dEcc_dsigma = +zb*mc*cos(sigma_0)*xc_1*rho_1 - zb*mc*sin(sigma_0)*xc_1*beta_1 - ...
                (-yb*sin(sigma_0) + xb*cos(sigma_0))*mc*xc_1*sigma_1 + ...
                zb*mc*sin(sigma_0)*yc_1*rho_1 + zb*mc*cos(sigma_0)*yc_1*beta_1 - ...
                (yb*cos(sigma_0) + xb*sin(sigma_0))*mc*yc_1*sigma_1;
            
            dEcrv_fl = approx_wheel_kin_energy_derivatives(mr_fl, xc_1, yc_1, rho_0, beta_0, sigma_0, rho_1, beta_1, sigma_1, p_fl, s_fl, d_fl, h_fl, l_fl_0);
            dEcrv_fr = approx_wheel_kin_energy_derivatives(mr_fr, xc_1, yc_1, rho_0, beta_0, sigma_0, rho_1, beta_1, sigma_1, p_fr, s_fr, d_fr, h_fr, l_fr_0);
            dEcrv_rl = approx_wheel_kin_energy_derivatives(mr_rl, xc_1, yc_1, rho_0, beta_0, sigma_0, rho_1, beta_1, sigma_1, p_rl, s_rl, d_rl, h_rl, l_rl_0);
            dEcrv_rr = approx_wheel_kin_energy_derivatives(mr_rr, xc_1, yc_1, rho_0, beta_0, sigma_0, rho_1, beta_1, sigma_1, p_rr, s_rr, d_rr, h_rr, l_rr_0);
            
            dEcc_dq = zeros(10, 1);
            dEcc_dq(6) = dEcc_dsigma + dEcrv_fl(1) + dEcrv_fr(1) + dEcrv_rl(1) + dEcrv_rr(1);
            dEcc_dq(7) = dEcrv_fl(2);
            dEcc_dq(8) = dEcrv_fr(2);
            dEcc_dq(9) = dEcrv_rl(2);
            dEcc_dq(10) = dEcrv_rr(2);
            
            %% Gravitational contribution
            % Car body
            g = 9.806;
            dVgc_dq_T = [0; 0; 1; (xb*sin(beta_0)*sin(rho_0) + yb*cos(rho_0) - zb*sin(rho_0)*cos(beta_0)); ...
                (-xb*cos(beta_0)*cos(rho_0) - zb*cos(rho_0)*sin(beta_0)); 0; 0; 0; 0; 0].*mc.*g;
            
            % Wheels
            % dVgr_dqr = grav_energy_deriv_wheel(p, s, d, h, l, rho, beta, sigma)
            dVgr_fl_dqr = mr_fl*g*grav_energy_deriv_wheel(p_fl, s_fl, d_fl, h_fl, l_fl_0, rho_0, beta_0, sigma_0);
            dVgr_fr_dqr = mr_fr*g*grav_energy_deriv_wheel(p_fr, s_fr, d_fr, h_fr, l_fr_0, rho_0, beta_0, sigma_0);
            dVgr_rl_dqr = mr_rl*g*grav_energy_deriv_wheel(p_rl, s_rl, d_rl, h_rl, l_rl_0, rho_0, beta_0, sigma_0);
            dVgr_rr_dqr = mr_rr*g*grav_energy_deriv_wheel(p_rr, s_rr, d_rr, h_rr, l_rr_0, rho_0, beta_0, sigma_0);
            dVgr_dqr_sm = dVgr_fl_dqr + dVgr_fr_dqr + dVgr_rl_dqr + dVgr_rr_dqr;
            dVgr_dq_T = [dVgr_dqr_sm(1:6); dVgr_fl_dqr(7); dVgr_fr_dqr(7); dVgr_rl_dqr(7); dVgr_rr_dqr(7)];
            
            % total
            dVg_dq_T = dVgc_dq_T + dVgr_dq_T;
            
            %% Elastic contribution
            % Suspension ("3/5/20 - 3")
            dVkl_fl_qs = ks_fl*(l_fl_0 - l_fl_ind);
            dVkl_fr_qs = ks_fr*(l_fr_0 - l_fr_ind);
            dVkl_rl_qs = ks_rl*(l_rl_0 - l_rl_ind);
            dVkl_rr_qs = ks_rr*(l_rr_0 - l_rr_ind);
            dVkl_dq_T = [zeros(6, 1); dVkl_fl_qs; dVkl_fr_qs; dVkl_rl_qs; dVkl_rr_qs];
            
            % Tyre/Wheel
            % Calculating support coordinates r and speeds
            r_fl_v = calc_r(zc_0, zc_1, zpn_fl_0, zpn_fl_1, p_fl, s_fl, d_fl, h_fl, l_fl_0, l_fl_1, rho_0, beta_0, rho_1, beta_1);
            r_fr_v = calc_r(zc_0, zc_1, zpn_fr_0, zpn_fr_1, p_fr, s_fr, d_fr, h_fr, l_fr_0, l_fr_1, rho_0, beta_0, rho_1, beta_1);
            r_rl_v = calc_r(zc_0, zc_1, zpn_rl_0, zpn_rl_1, p_rl, s_rl, d_rl, h_rl, l_rl_0, l_rl_1, rho_0, beta_0, rho_1, beta_1);
            r_rr_v = calc_r(zc_0, zc_1, zpn_rr_0, zpn_rr_1, p_rr, s_rr, d_rr, h_rr, l_rr_0, l_rr_1, rho_0, beta_0, rho_1, beta_1);
            r_fl_0 = r_fl_v(1); r_fl_1 = r_fl_v(2);
            r_fr_0 = r_fr_v(1); r_fr_1 = r_fr_v(2);
            r_rl_0 = r_rl_v(1); r_rl_1 = r_rl_v(2);
            r_rr_0 = r_rr_v(1); r_rr_1 = r_rr_v(2);
            
            % Tyre spring derivative contribution.
            dVkp_fl_dq = tyre_spring_potential_derivative(kp_fl, r_fl_0, r_fl_ind, p_fl, s_fl, d_fl, h_fl, l_fl_0, rho_0, beta_0, zc_0, zpn_fl_0);
            dVkp_fr_dq = tyre_spring_potential_derivative(kp_fr, r_fr_0, r_fr_ind, p_fr, s_fr, d_fr, h_fr, l_fr_0, rho_0, beta_0, zc_0, zpn_fr_0);
            dVkp_rl_dq = tyre_spring_potential_derivative(kp_rl, r_rl_0, r_rl_ind, p_rl, s_rl, d_rl, h_rl, l_rl_0, rho_0, beta_0, zc_0, zpn_rl_0);
            dVkp_rr_dq = tyre_spring_potential_derivative(kp_rr, r_rr_0, r_rr_ind, p_rr, s_rr, d_rr, h_rr, l_rr_0, rho_0, beta_0, zc_0, zpn_rr_0);
            
            % Assembling
            dVkp_dq_T = [dVkp_fl_dq(1:6, 1) + dVkp_fr_dq(1:6, 1) + dVkp_rl_dq(1:6, 1) + dVkp_rr_dq(1:6, 1);
                dVkp_fl_dq(7, 1); dVkp_fr_dq(7, 1); dVkp_rl_dq(7, 1); dVkp_rr_dq(7, 1)];
            
            dV_dq_T = dVg_dq_T + dVkl_dq_T + dVkp_dq_T;
            
            %% Viscous damping
            % Suspension ("3/5/20 - 7")
            dDs_fl_qs = rs_fl*(l_fl_1);
            dDs_fr_qs = rs_fr*(l_fr_1);
            dDs_rl_qs = rs_rl*(l_rl_1);
            dDs_rr_qs = rs_rr*(l_rr_1);
            dDs_dq_T = [zeros(6, 1); dDs_fl_qs; dDs_fr_qs; dDs_rl_qs; dDs_rr_qs];
            
            % Tyres ("8/5/20 -2")
            % dDp_dq = tyre_damper_derivative(rp, r_0, r_1, p, s, d, h, l, rho, beta)
            dDp_fl_dq = tyre_damper_derivative(rp_fl, r_fl_0, r_fl_1, p_fl, s_fl, d_fl, h_fl, l_fl_0, rho_0, beta_0);
            dDp_fr_dq = tyre_damper_derivative(rp_fr, r_fr_0, r_fr_1, p_fr, s_fr, d_fr, h_fr, l_fr_0, rho_0, beta_0);
            dDp_rl_dq = tyre_damper_derivative(rp_rl, r_rl_0, r_rl_1, p_rl, s_rl, d_rl, h_rl, l_rl_0, rho_0, beta_0);
            dDp_rr_dq = tyre_damper_derivative(rp_rr, r_rr_0, r_rr_1, p_rr, s_rr, d_rr, h_rr, l_rr_0, rho_0, beta_0);
            
            % Assembling
            dDp_dq_T = [dDp_fl_dq(1:6, 1) + dDp_fr_dq(1:6, 1) + dDp_rl_dq(1:6, 1) + dDp_rr_dq(1:6, 1);
                dDp_fl_dq(7, 1); dDp_fr_dq(7, 1); dDp_rl_dq(7, 1); dDp_rr_dq(7, 1)];
            
            dD_dq_T = dDs_dq_T + dDp_dq_T;
            
            %% External forces
            % Still unsure whether to put tyre moments. Wait until I think about it.
            % EDIT 24/05/2020
            % Trying to calculate forces according to Pacejka.
            
            % Calc vertical forces for the wheels
            Fz_r_fl = -kp_fl*(r_fl_0 - r_fl_ind) - rp_fl*(r_fl_1);
            Fz_r_fr = -kp_fr*(r_fr_0 - r_fr_ind) - rp_fr*(r_fr_1);
            Fz_r_rl = -kp_rl*(r_rl_0 - r_rl_ind) - rp_rl*(r_rl_1);
            Fz_r_rr = -kp_rr*(r_rr_0 - r_rr_ind) - rp_rr*(r_rr_1);
            
            if obj.apply_tyre_forces 
                [Fx_t_fl, Fy_t_fl, Mz_t_fl, slip_rate_fl, slip_angle_fl_deg] = apply_pacejka(Fz_r_fl, phi_fl, p_fl, ...
                    s_fl, d_fl, h_fl, l_fl_0, l_fl_1, rr_fl, xc_1, yc_1, zc_1, rho_0, ...
                    beta_0, sigma_0, rho_1, beta_1, sigma_1, theta_fl_1, tyre_param_fl, 0);
                [Fx_t_fr, Fy_t_fr, Mz_t_fr, slip_rate_fr, slip_angle_fr_deg] = apply_pacejka(Fz_r_fr, phi_fr, p_fr, ...
                    s_fr, d_fr, h_fr, l_fr_0, l_fr_1, rr_fr, xc_1, yc_1, zc_1, rho_0, ...
                    beta_0, sigma_0, rho_1, beta_1, sigma_1, theta_fr_1, tyre_param_fr, 1);
                [Fx_t_rl, Fy_t_rl, Mz_t_rl, slip_rate_rl, slip_angle_rl_deg] = apply_pacejka(Fz_r_rl, phi_rl, p_rl, ...
                    s_rl, d_rl, h_rl, l_rl_0, l_rl_1, rr_rl, xc_1, yc_1, zc_1, rho_0, ...
                    beta_0, sigma_0, rho_1, beta_1, sigma_1, theta_rl_1, tyre_param_rl, 0);
                [Fx_t_rr, Fy_t_rr, Mz_t_rr, slip_rate_rr, slip_angle_rr_deg] = apply_pacejka(Fz_r_rr, phi_rr, p_rr, ...
                    s_rr, d_rr, h_rr, l_rr_0, l_rr_1, rr_rr, xc_1, yc_1, zc_1, rho_0, ...
                    beta_0, sigma_0, rho_1, beta_1, sigma_1, theta_rr_1, tyre_param_rr, 1);
                % (Here, forces calculated with no drivetrain. Amarcord.)
                %theta_fl_2 = -(Fx_t_fl * rr_fl - 2.5e-4*theta_fl_1) / Jr_fl;
                %theta_fr_2 = -(Fx_t_fr * rr_fr - 2.5e-4*theta_fr_1) / Jr_fr;
                %theta_rl_2 = -(Fx_t_rl * rr_rl - 2.5e-4*theta_rl_1) / Jr_rl;
                %theta_rr_2 = -(Fx_t_rr * rr_rr - 2.5e-4*theta_rr_1) / Jr_rr;
            else
                Fx_t_fl = 0; Fy_t_fl = 0; Mz_t_fl = 0;
                Fx_t_fr = 0; Fy_t_fr = 0; Mz_t_fr = 0;
                Fx_t_rl = 0; Fy_t_rl = 0; Mz_t_rl = 0;
                Fx_t_rr = 0; Fy_t_rr = 0; Mz_t_rr = 0;
                slip_rate_fl = 0; slip_rate_fr = 0; slip_rate_rl = 0; slip_rate_rr = 0;
                slip_angle_fl_deg = 0; slip_angle_fr_deg = 0; slip_angle_rl_deg = 0; slip_angle_rr_deg = 0;
                %theta_fl_2 = 0; theta_fr_2 = 0; theta_rl_2 = 0; theta_rr_2 = 0;
            end
            
            %% Drivetrain does his thing.
            % FL-RL-FR-RR convention (retro-compatibility).
            w0v = [theta_m_1; theta_c_1; theta_fl_1; theta_rl_1; theta_fr_1; theta_rr_1];
            [w1v, dap] = obj.drivetrain.get_shaft_acc(w0v, Fx_t_fl, Fx_t_rl, Fx_t_fr, Fx_t_rr);
            
            % Re.bring forces in the vehicle's frame.
            Fxr_fl = Fx_t_fl*cos(phi_fl) - Fy_t_fl*sin(phi_fl);
            Fyr_fl = Fx_t_fl*sin(phi_fl) + Fy_t_fl*cos(phi_fl);
            Fxr_fr = Fx_t_fr*cos(phi_fr) - Fy_t_fr*sin(phi_fr);
            Fyr_fr = Fx_t_fr*sin(phi_fr) + Fy_t_fr*cos(phi_fr);
            Fxr_rl = Fx_t_rl*cos(phi_rl) - Fy_t_rl*sin(phi_rl);
            Fyr_rl = Fx_t_rl*sin(phi_rl) + Fy_t_rl*cos(phi_rl);
            Fxr_rr = Fx_t_rr*cos(phi_rr) - Fy_t_rr*sin(phi_rr);
            Fyr_rr = Fx_t_rr*sin(phi_rr) + Fy_t_rr*cos(phi_rr);
            
            % Longitudinal and lateral forces acting onto tyre-road interface
            % Q = reproject_tyre_forces(Fx, Fy, p, s, d, h, l, r, rho, beta, sigma)
            %Qp_fl = reproject_tyre_forces(Fx_t_fl, Fy_t_fl, p_fl, s_fl, d_fl, h_fl, l_fl_0, r_fl_0, rho_0, beta_0, sigma_0, zc_0, zpn_fl_0)';
            %Qp_fr = reproject_tyre_forces(Fx_t_fr, Fy_t_fr, p_fr, s_fr, d_fr, h_fr, l_fr_0, r_fr_0, rho_0, beta_0, sigma_0, zc_0, zpn_fr_0)';
            %Qp_rl = reproject_tyre_forces(Fx_t_rl, Fy_t_rl, p_rl, s_rl, d_rl, h_rl, l_rl_0, r_rl_0, rho_0, beta_0, sigma_0, zc_0, zpn_rl_0)';
            %Qp_rr = reproject_tyre_forces(Fx_t_rr, Fy_t_rr, p_rr, s_rr, d_rr, h_rr, l_rr_0, r_rr_0, rho_0, beta_0, sigma_0, zc_0, zpn_rr_0)';
            Qp_fl = reproject_tyre_forces(Fxr_fl, Fyr_fl, p_fl, s_fl, d_fl, h_fl, l_fl_0, r_fl_0, rho_0, beta_0, sigma_0, zc_0, zpn_fl_0)';
            Qp_fr = reproject_tyre_forces(Fxr_fr, Fyr_fr, p_fr, s_fr, d_fr, h_fr, l_fr_0, r_fr_0, rho_0, beta_0, sigma_0, zc_0, zpn_fr_0)';
            Qp_rl = reproject_tyre_forces(Fxr_rl, Fyr_rl, p_rl, s_rl, d_rl, h_rl, l_rl_0, r_rl_0, rho_0, beta_0, sigma_0, zc_0, zpn_rl_0)';
            Qp_rr = reproject_tyre_forces(Fxr_rr, Fyr_rr, p_rr, s_rr, d_rr, h_rr, l_rr_0, r_rr_0, rho_0, beta_0, sigma_0, zc_0, zpn_rr_0)';
            
            Qp = [Qp_fl(1:6, 1) + Qp_fr(1:6, 1) + Qp_rl(1:6, 1) + Qp_rr(1:6, 1);
                Qp_fl(7, 1); Qp_fr(7, 1); Qp_rl(7, 1); Qp_rr(7, 1)];
            
            
            
            % Aerodynamic forces (from "8/5/20 - 6");
            % Drag:
            rho = 1.22; % air density
            Cx = obj.vg.Cx;
            S = obj.vg.S;
            Fd_x = -.5*rho*Cx*S*xc_1*abs(xc_1);
            Fd_y = -.5*rho*Cx*S*yc_1*abs(yc_1);
            Q_aer_d = [Fd_x; Fd_y; zeros(8, 1)];
            
            % Lift
            % TBD. (Formulas are there).
            Q_aer_l = zeros(10, 1);
            
            Q = Qp + Q_aer_d + Q_aer_l;
            % Q = [Qp(1); zeros(9, 1)];
            % disp(Q(1));
            % Q(2:end) = -Q(2:end);
            
            %% Assembling final equation
            DU = -M1*q_1 + dEcc_dq - dV_dq_T - dD_dq_T + Q;
            q_2 = M \ DU;
            
            A_car = L_c_0 \ q_2(1:3); % relative car accelerations (car reference).
            
            par_out = dap;
            %par_out.phiv = [phi_fl, phi_fr, phi_rl, phi_rr];
            par_out.A_car = A_car;
            par_out.Ax = A_car(1); par_out.Ay = A_car(2); par_out.Az = A_car(3);
            %par_out.q_2 = q_2;
            %par_out.q_1 = q_1;
            %par_out.q_0 = q_0;
            %par_out.M = M;
            %par_out.dEcc_dq = dEcc_dq;
            %par_out.dV_dq_T = dV_dq_T;
            %par_out.dD_dq_T = dD_dq_T;
            %par_out.Q = Q;
            par_out.Fz = [Fz_r_fl, Fz_r_fr, Fz_r_rl, Fz_r_rr];
            par_out.Fx = [Fx_t_fl, Fx_t_fr, Fx_t_rl, Fx_t_rr];
            par_out.Fy = [Fy_t_fl, Fy_t_fr, Fy_t_rl, Fy_t_rr];
            %par_out.r_wheels = [r_fl_0, r_fr_0, r_rl_0, r_rr_0];
            %par_out.rv_wheels = [r_fl_1, r_fr_1, r_rl_1, r_rr_1];
            %par_out.F_aer_x = Fd_x;
            %par_out.F_aer_y = Fd_y;
            par_out.kv = [slip_rate_fl, slip_rate_fr, slip_rate_rl, slip_rate_rr];
            par_out.sa = [slip_angle_fl_deg, slip_angle_fr_deg, slip_angle_rl_deg, slip_angle_rr_deg];
            
            % Sad choice of FL-RL-FR-RR convention for
            % retrocompatibility...
            theta_2_v = [w1v(1); w1v(2); w1v(3); w1v(5); w1v(4); w1v(6)];
            
            %xd = [q_2; theta_2_v; y(1:14)];
            x2v = [theta_2_v; q_2];
            %% UNTESTED!
        end
        
    end
    
    
end
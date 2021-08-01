classdef Drivetrain2Axle_v2 < handle
    % Drivetrain for a 2 axle vehicle, supporting one engine, one clutch,
    % one gearbox, an open or closed diff, a brake distributor acting onto
    % two brakes, etc.
    
    properties
        engine = [];
        clutch = [];
        gearbox = [];
        differential = [];
        brake_system = [];
        axles = [];
        controls = [];
        wheel_front = [];
        wheel_rear = [];
        brake_speed_tol = 0.01;
    end
    
    methods
        
        function obj = Drivetrain2Axle_v2(n_engine, n_clutch, n_gearbox, n_differential, ...
                n_brake_system, n_controls, n_wheel_front, n_wheel_rear)
            obj.engine = n_engine;
            obj.clutch = n_clutch;
            obj.gearbox = n_gearbox;
            obj.differential = n_differential;
            obj.brake_system = n_brake_system;
            obj.controls = n_controls;
            obj.wheel_front = n_wheel_front;
            obj.wheel_rear = n_wheel_rear;
        end
        
        
        % Returns shaft accelerations given shaft speeds (all independent
        % rotating parts in the vehicle).
        % Engine - Driveshaft - Front axle - Rear axle
        function [w1v, add_params] = get_shaft_acc(obj, w0v, Fx_fl, Fx_rl, Fx_fr, Fx_rr)
            if nargin < 5 % retro-compatibility
                Fx_fl = Fx_fl / 2;
                Fx_rl = Fx_rl / 2;
                Fx_fr = Fx_fl;
                Fx_rr = Fx_rl;
                cut_output = true;
            else
                cut_output = false;
            end
            th1_m = w0v(1); % engine speed
            th1_c = w0v(2); % clutch speed
            % Order of wheel speeds is defined like this to preserve
            % retro-compatibility.
            th1_fl = w0v(3); % speed of front left wheel
            th1_rl = w0v(4); % speed of rear left wheel
            if length(w0v) > 4
                th1_fr = w0v(5); % FR
                th1_rr = w0v(6); % RR
            else
                th1_fr = th1_fl;
                th1_rr = th1_rl;
            end
            c_gear = obj.controls.gear_lever;
            if c_gear == 0 % Neutral
                t = 50.0; % Random value, really
                clutch_is_engaged = false;
            else
                if c_gear == -1
                    t_gbox = obj.gearbox.get_ratio(0);
                else
                    t_gbox = obj.gearbox.get_ratio(c_gear);
                end
                t_diff = obj.differential.final_drive;
                t = t_gbox * t_diff;
                obj.clutch.update_coupled_state(obj.controls.clc_pedal, th1_m, th1_c);
                clutch_is_engaged = obj.clutch.is_coupled;
            end
            if clutch_is_engaged % compute reduced Jc and rc.
                Jc_a = obj.clutch.J + obj.engine.Jm;
                rc_a = obj.clutch.r + obj.engine.fv_0;
                Mc_a = -obj.engine.get_torque(obj.controls.gas_pedal, rads2rpm(th1_m));
            else % separately handle engine and clutch case
                %clutch_factor = obj.clutch.get_engaged_factor(obj.controls.clc_pedal);
                %Mc_a = -obj.clutch.kf*clutch_factor*sign(th1_m - th1_c);
                if c_gear == 0 % neutral
                    Mc_a = 0;
                else
                    Mc_a = obj.clutch.get_torque(th1_m, th1_c, obj.controls.clc_pedal);
                end
                th2_m = (obj.engine.get_torque(obj.controls.gas_pedal, rads2rpm(th1_m)) +Mc_a - obj.engine.fv_0*th1_m) / obj.engine.Jm;
                rc_a = obj.clutch.r;
                Jc_a = obj.clutch.J;
            end
            th1_d = th1_c / t;
            th1_v = [th1_fl; th1_fr; th1_rl; th1_rr];
            [M_brk_max_front_abs, M_brk_max_rear_abs] = obj.brake_system.get_brake_torques();
            chb = false;
            if abs(th1_fl) > obj.brake_speed_tol
                M_brk_max_fl = M_brk_max_front_abs * (sign(th1_fl))/2; % new convention: positive when braking.
            else
                M_brk_max_fl = M_brk_max_front_abs/2;
                chb = true;
            end
            if abs(th1_fr) > obj.brake_speed_tol
                M_brk_max_fr = M_brk_max_front_abs * (sign(th1_fr))/2;
            else
                M_brk_max_fr = M_brk_max_front_abs/2;
                chb = true;
            end
            if abs(th1_rl) > obj.brake_speed_tol
                M_brk_max_rl  = M_brk_max_rear_abs * (sign(th1_rl))/2;
            else
                M_brk_max_rl = M_brk_max_rear_abs/2;
                chb = true;
            end
            if abs(th1_rr) > obj.brake_speed_tol
                M_brk_max_rr  = M_brk_max_rear_abs * (sign(th1_rr))/2;
            else
                M_brk_max_rr = M_brk_max_rear_abs/2;
                chb = true;
            end
            Mfv = [M_brk_max_fl; M_brk_max_fr; M_brk_max_rl; M_brk_max_rr];
            Fxv = [Fx_fl; Fx_fr; Fx_rl; Fx_rr];
            Jrv = [obj.wheel_front.J; obj.wheel_front.J; obj.wheel_rear.J; obj.wheel_rear.J];
            Rv = [obj.wheel_front.R; obj.wheel_front.R; obj.wheel_rear.R; obj.wheel_rear.R];
            [th2_v, th2d, M_v, Md, Mda, Mdp, add_param] = drivetrain_gen(obj.differential.type, ...
                th1_v, th1_d, Mc_a, Mfv, Fxv, Jc_a, Jrv, rc_a, obj.differential.r, t, ...
                obj.differential.torque_distribution(1), obj.differential.torque_distribution(2), ...
                Rv, chb);
            if clutch_is_engaged
                th2_m = t*th2d;
            end % else already calculated
            w1v = [th2_m; t*th2d; th2_v(1); th2_v(3); th2_v(2); th2_v(4)]; % same FL-RL-FR-RR for retrocomp.
            if cut_output
                w1v(5:6) = [];
            end
            add_params = struct();
            add_params.M_v = M_v;
            add_params.Md = Md;
            add_params.Mda = Mda;
            add_params.Mdp = Mdp;
            add_params.Mb = add_param.Mfv;
            add_params.Mfv_th = Mfv;
            add_params.w_diff = add_param.th1d;
            add_params.w_clutch = t*add_params.w_diff;
            add_params.ClutchTorque = -Mc_a;
            add_params.EngineTorque = obj.engine.get_torque(obj.controls.gas_pedal, rads2rpm(th1_m));
            add_params.WheelTorques = M_v;
            add_params.DiffTorque = Md;
            add_params.BrakeTorquesActual = add_param.Mfv;
            add_params.Mfv_eff = add_params.BrakeTorquesActual;
            add_params.BrakeTorquesTheor = Mfv;
            add_params.ClutchEngaged = clutch_is_engaged;
        end
        
    end
    
end
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
        function [w1v, add_params] = get_shaft_acc(obj, w0v, Fx_front, Fx_rear)
            th1_m = w0v(1); % engine speed
            th1_c = w0v(2); % clutch speed
            th1_f = w0v(3); % speed of front wheel
            th1_r = w0v(4); % speed of rear wheel
            c_gear = obj.controls.gear_lever;
            t_gbox = obj.gearbox.get_ratio(c_gear);
            t_diff = obj.differential.final_drive;
            t = t_gbox * t_diff;
            obj.clutch.update_coupled_state(obj.controls.clc_pedal, th1_m, th1_c);
            clutch_is_engaged = obj.clutch.is_coupled;
            if clutch_is_engaged % compute reduced Jc and rc.
                Jc_a = obj.clutch.J + obj.engine.Jm;
                rc_a = obj.clutch.r + obj.engine.fv_0;
                Mc_a = -obj.engine.get_torque(obj.controls.gas_pedal, rads2rpm(th1_m));
            else % separately handle engine and clutch case
                clutch_factor = obj.clutch.get_engaged_factor(obj.controls.clc_pedal);
                Mc_a = -obj.clutch.kf*clutch_factor*(th1_m - th1_c);
                th2_m = (obj.engine.get_torque(obj.controls.gas_pedal, rads2rpm(th1_m)) +Mc_a - obj.engine.fv_0*th1_m) / obj.engine.Jm;
                rc_a = obj.clutch.r;
                Jc_a = obj.clutch.J;
            end
            th1_d = th1_c / t;
            th1_v = [th1_f; th1_f; th1_r; th1_r];
            [M_brk_max_front_abs, M_brk_max_rear_abs] = obj.brake_system.get_brake_torques();
            chb = false;
            if abs(th1_f) > obj.brake_speed_tol
                M_brk_max_front = M_brk_max_front_abs * (sign(th1_f)); % new convention: positive when braking.
            else
                M_brk_max_front = M_brk_max_front_abs;
                chb = true;
            end
            if abs(th1_r) > obj.brake_speed_tol
                M_brk_max_rear  = M_brk_max_rear_abs * (sign(th1_r));
            else
                M_brk_max_rear = M_brk_max_rear_abs;
                chb = true;
            end
            Mfv = [M_brk_max_front/2; M_brk_max_front/2; M_brk_max_rear/2; M_brk_max_rear/2];
            Fxv = [Fx_front/2; Fx_front/2; Fx_rear/2; Fx_rear/2];
            Jrv = [obj.wheel_front.J/2; obj.wheel_front.J/2; obj.wheel_rear.J/2; obj.wheel_rear.J/2];
            Rv = [obj.wheel_front.R; obj.wheel_front.R; obj.wheel_rear.R; obj.wheel_rear.R];
            [th2_v, th2d, M_v, Md, Mda, Mdp, add_param] = drivetrain_gen(obj.differential.type, ...
                th1_v, th1_d, Mc_a, Mfv, Fxv, Jc_a, Jrv, rc_a, obj.differential.r, t, ...
                obj.differential.torque_distribution(1), obj.differential.torque_distribution(2), ...
                Rv, chb);
            if clutch_is_engaged
                th2_m = t*th2d;
            end % else already calculated
            w1v = [th2_m; t*th2d; th2_v(1); th2_v(3)];
            add_params = struct();
            add_params.M_v = M_v;
            add_params.Md = Md;
            add_params.Mda = Mda;
            add_params.Mdp = Mdp;
            add_params.Mb = add_param.Mfv;
        end
        
    end
    
end
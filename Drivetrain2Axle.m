classdef Drivetrain2Axle < handle
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
    end
    
    methods
        
        function obj = Drivedtrain2Axle(n_engine, n_clutch, n_gearbox, n_differential, n_brake_system, n_axles, n_controls)
            obj.engine = n_engine;
            obj.clutch = n_clutch;
            obj.gearbox = n_gearbox;
            obj.differential = n_differential;
            obj.brake_system = n_brake_system;
            obj.axles = n_axles;
            obj.controls = n_controls;
        end
        
        
        % Returns shaft accelerations given shaft speeds (all independent
        % rotating parts in the vehicle).
        % Engine - Driveshaft - Front axle - Rear axle
        function w1v = get_shaft_acc(obj, w0v, M_res_front, M_res_rear)
            % Extract speeds from input vector
            w0_engine = w0v(1); % measured at engine shaft
            w0_driveshaft = w0v(2); % a valle del cambio, a monte del diff.
            w0_front = w0v(3);
            w0_rear = w0v(4);
            % Find if tyres are moving
            obj.axles(1).update_still_situation(w0_front);
            obj.axles(2).update_still_situation(w0_rear);
            % Update clutch state
            curr_gear = obj.controls.gear_lever;
            curr_gear_ratio = obj.gearbox.get_ratio(curr_gear);
            w0_clutch = w0_driveshaft * curr_gear_ratio;
            obj.clutch.update_coupled_state(obj.controls.clc_pedal, w0_engine, w0_clutch);
            clutch_engaged = obj.clutch.is_coupled;
            % Get maximum possible braking force per each axle
            [M_brk_max_front_abs, M_brk_max_rear_abs] = obj.brake_system.get_brake_torques();
            M_brk_max_front = M_brk_max_front_abs * (-sign(w0_front));
            M_brk_max_rear  = M_brk_max_rear_abs * (-sign(w0_rear));
            % Calcolare queste in base a stato vettura e se sono reazioni
            M_tot_front = 0;
            M_tot_rear = 0;
            Mm = obj.engine.get_torque(obj.controls.gas_pedal, rads2rpm(w_engine));
            % Here, the almost-still or locked tyre is not considered... 
            if clutch_engaged % engine is IN the system
                Mds = Mm * (curr_gear_ratio * obj.differential.final_drive);
                Jds = ((curr_gear_ratio * obj.differential.final_drive)^2)*obj.engine.Jm;
                wds = w0_driveshaft / obj.differential.final_drive;
                wp = obj.differential.balance_function([wds, w0_front, w0_rear], ...
                    Jds, obj.axles(1).J, obj.axles(2).J, Mds, M_tot_front, M_tot_rear); % CHECK SIGNS! They are RESISTING torques...
                w1_driveshaft = wp(1) * obj.differential.final_drive;
                w1_engine = w1_driveshaft * curr_gear_ratio;
                w1_front = wp(2);
                w1_rear = wp(3);
            else
                clutch_factor = obj.clutch.get_engaged_factor(obj.controls.clc_pedal);
                M_sub = obj.clutch.kf*clutch_factor*(w_engine - w_clutch);
                Mds = M_sub * (curr_gear_ratio * obj.differential.final_drive);
                w1_engine = (M_engine - M_sub) / obj.engine.Jm;
                % Locked differential case must be understood separately.
                if obj.axles(1).still
                    M_brk_max_front_abs = 2*M_brk_max_front_abs;
                    if abs(Mds*obj.differential.torque_distribution(1) - M_res_front) < M_brk_max_front_abs % not moving
                        M_tot_front = Mds*obj.differential.torque_distribution(1) - M_res_front;
                    end
                end
            end
        end
        
    end
    
end
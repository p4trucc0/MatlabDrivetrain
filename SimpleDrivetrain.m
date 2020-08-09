classdef SimpleDrivetrain < handle
    % Dummy drivetrain class.
    % It will need serious restructuring once a proper vehicle architecture
    % is defined (more than 2WD, proper axes, etc).
    
    properties
        engine = [];
        clutch = [];
        gearbox = [];
        differential = [];
        controls = [];
    end
    
    methods
        
        function obj = SimpleDrivetrain(n_engine, n_clutch, n_gearbox, n_differential, n_controls)
            obj.engine = n_engine;
            obj.clutch = n_clutch;
            obj.gearbox = n_gearbox;
            obj.differential = n_differential;
            obj.controls = n_controls;
        end
        
        % Returns shaft rotational acceleration for both engine and
        % clutch/gearbox.
        function get_shaft_acc(obj, w_diff, w_engine, M_diff, J_diff) % takes as input speeds of differential and of engine, and torque applied to the differential
            % First, determine whether clutch is engaged
            w_gear = w_diff * obj.differential.final_drive;
            curr_gear = obj.controls.gear_lever;
            curr_gear_ratio = obj.gearbox.get_ratio(curr_gear);
            w_clutch = w_gear * curr_gear_ratio;
            obj.clutch.update_coupled_state(obj.controls.clc_pedal, w_engine, w_clutch);
            clutch_engaged = obj.clutch.is_coupled;
            M_clutch = (M_diff / obj.differential.final_drive / curr_gear_ratio) * obj.gearbox.get_efficiency(curr_gear);
            M_engine = obj.engine.get_max_torque(obj.controls.gas_pedal, rads2rpm(w_engine)); % TODO: Change this when a more sophisticated engine is available.
            if clutch_engaged % one balance
                J_tot = J_diff + obj.engine.Jm; % total moment of inertia
                w1_engine = (M_engine + M_clutch) / J_tot; % engine acceleration. CHECK SIGNS.
                w1_diff = w1_engine / curr_gear_ratio / obj.differential.final_drive;
            else % two separate sub-systems
                % Balance without inertias
                clutch_factor = obj.clutch.get_engaged_factor(obj.controls.clc_pedal);
                M_delta_clutch = M_engine - M_clutch;
                M_sub_engine = - M_delta_clutch * clutch_factor;
                M_sub_clutch = M_delta_clutch * clutch_factor;
                w1_engine = (M_engine + M_sub_engine) / obj.engine.Jm;
                w1_clutch = (M_clutch + M_sub_clutch) / J_diff;
                w1_diff = w1_clutch / curr_gear_ratio / obj.differential.final_drive;
            end
        end
        
        
    end
    
    
    
end
classdef SimpleDrivetrain < handle
    % Dummy drivetrain class.
    % It will need serious restructuring once a proper vehicle architecture
    % is defined (more than 2WD, proper axes, etc).
    
    properties
        engine = [];
        clutch = [];
        gearbox = [];
        differential = [];
        brake = [];
        controls = [];
    end
    
    methods
        
        function obj = SimpleDrivetrain(n_engine, n_clutch, n_gearbox, n_differential, n_brake, n_controls)
            obj.engine = n_engine;
            obj.clutch = n_clutch;
            obj.gearbox = n_gearbox;
            obj.differential = n_differential;
            obj.brake = n_brake;
            obj.controls = n_controls;
        end
        
        % Returns shaft rotational acceleration for both engine and
        % clutch/gearbox.
        function [w1_engine, w1_diff] = get_shaft_acc(obj, w_diff, w_engine, M_diff, J_diff, dt) % takes as input speeds of differential and of engine, and torque applied to the differential
            % Calculate maximum current braking torque.
            if nargin < 6
                dt = .01;
            end
            M_brake_max = obj.brake.get_simple_brake_torque(obj.controls.brk_pedal);
            M_brake_wheel = -M_brake_max * sign(w_diff); % always opposes speed.
            if M_brake_wheel == 0
                M_brake_wheel = M_brake_max;
            end
            if abs(w_diff) > .1 % tyre not still
                tyre_is_still = false;
            else
                tyre_is_still = true;
            end
            if abs(w_diff) > 5 % tyre not still
                close_to_still = false;
            else
                close_to_still = true;
            end
            w_gear = w_diff * obj.differential.final_drive;
            curr_gear = obj.controls.gear_lever;
            curr_gear_ratio = obj.gearbox.get_ratio(curr_gear);
            w_clutch = w_gear * curr_gear_ratio;
            obj.clutch.update_coupled_state(obj.controls.clc_pedal, w_engine, w_clutch);
            clutch_engaged = obj.clutch.is_coupled;
            M_clutch = (M_diff / obj.differential.final_drive / curr_gear_ratio) * obj.gearbox.get_efficiency(curr_gear);
            M_brake  = (M_brake_wheel / obj.differential.final_drive / curr_gear_ratio) * obj.gearbox.get_efficiency(curr_gear);
            M_engine = obj.engine.get_torque(obj.controls.gas_pedal, rads2rpm(w_engine)); % add efficiency here!
            if clutch_engaged % one balance
                J_tot = J_diff + obj.engine.Jm; % total moment of inertia
                w1_engine = (M_engine + M_clutch + M_brake) / J_tot; % engine acceleration. CHECK SIGNS.
                w1_diff = w1_engine / curr_gear_ratio / obj.differential.final_drive;
            else % two separate sub-systems
                % Balance without inertias
                clutch_factor = obj.clutch.get_engaged_factor(obj.controls.clc_pedal);
                M_sub = obj.clutch.kf*clutch_factor*(w_engine - w_clutch);
                w1_engine = (M_engine - M_sub) / obj.engine.Jm;
                if ~tyre_is_still
                    w1_clutch = (M_clutch + M_sub + M_brake) / J_diff;
                    if close_to_still % approaching zero speed.
                        if abs(M_brake_wheel) > 100 % braking in a consistent way
                            if sign(w1_clutch)*sign(w_clutch) == -1 % reducing absolute wheel speed
                                if sign(w_clutch)*(w_clutch + w1_clutch*dt) < 0
                                    w1_clutch = -w_clutch / dt; % bring speed to zero.
                                end
                            end
                        end
                    end
                else
                    M_brake = 2*M_brake; % static friction coefficient.
                    if (abs(M_sub + M_clutch) > abs(M_brake)) %|| (sign(M_sub + M_clutch)*sign(M_brake) == 1)
                        w1_clutch = (M_clutch + M_sub + M_brake) / J_diff;
                    else
                        % w1_clutch = -w_clutch / dt; % bring speed to zero.
                        w1_clutch = 0;
                    end
                end
                w1_diff = w1_clutch / curr_gear_ratio / obj.differential.final_drive;
            end
            if isnan(w1_engine)
                keyboard;
            end
        end
        
        
    end
    
    
    
end
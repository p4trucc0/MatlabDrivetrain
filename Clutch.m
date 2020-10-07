classdef Clutch < handle
    
    properties
        is_coupled = false; % if true, the clutch behaves as a simple rod.
        ef_thr = .99; % if greater than this, consider engine fully engaged
        sp_thr = .98; % max speed differential between engine and gearbox
        kf = .1;
        J = 0.1;
        r = 0.0;
    end
    
    methods
        
        function obj = Clutch(n_kf, n_J, n_r)
            if nargin < 1
                obj.kf = .1;
            else
                obj.kf = n_kf;
                if nargin > 1
                    obj.J = n_J;
                    obj.r = n_r;
                else
                    obj.J = 0.1;
                    obj.r = 0.0;
                end
            end
        end
        
        % Dummy function returning an "engagement factor" which is, for the
        % time being, just equal to the pedal position
        % In a better revision, this should take into account torque
        % difference to see whether the pressure on the pedal is enough to
        % allow for speed difference between the two elements.
        function ef = get_engaged_factor(obj, cp_pos)
            ef = 1 - cp_pos;
        end
        
        function update_coupled_state(obj, cp_pos, w_engine, w_gearbox)
            engagement_factor = obj.get_engaged_factor(cp_pos);
            if engagement_factor < obj.ef_thr % I am pushing the pedal - not engaged.
                obj.is_coupled = false; 
            else
                if (((w_engine/w_gearbox) >= obj.sp_thr) && ...
                        ((w_gearbox/w_engine) >= obj.sp_thr)) 
                    obj.is_coupled = true;
                else % engaged, but there is speed difference: should wait for the two pieces to reach the same w.
                    obj.is_coupled = false;
                end
            end
        end
        
        
        % function tt = get_transmitted_torque(obj, cp_pos)
        % end
        
    end
    
end
classdef Differential2Axle < handle
    % Very simple (center) differential class, for now NOT supporting complex laws
    % regarding, for example, locking.
    
    properties
        final_drive = [];
        torque_distribution = [.5 .5];
        type = 'open';
        balance_function = [];
    end
    
    methods
        
        function obj = Differential2Axle(n_final_drive, n_torque_distribution, n_type)
            obj.final_drive = n_final_drive;
            obj.torque_distribution = n_torque_distribution;
            obj.type = n_type;
            if strcmp(obj.type, 'open')
                d_mode = 0;
            else
                d_mode = 1;
            end
            % TODO: should introduce final drive here.
            obj.balance_function = @(w0v, Jm, Jf, Jr, Mm, Mf, Mr) ...
                differential_function(w0v, Jm, Jf, Jr, Mm, Mf, Mr, d_mode, obj.torque_distribution);
        end
        
    end
    
end
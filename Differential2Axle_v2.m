classdef Differential2Axle_v2 < handle
    % Edit 07/10/2020
    % Much simpler than before: it does not come with a dedicated function,
    % but only with a type. 
    
    properties
        final_drive = [];
        torque_distribution = [.5 .5];
        r = 0.0;
        type = 4; %FR
    end
    
    methods
        
        function obj = Differential2Axle_v2(n_final_drive, n_torque_distribution, n_type, n_r)
            obj.final_drive = n_final_drive;
            obj.torque_distribution = n_torque_distribution;
            obj.type = n_type;
            obj.r = n_r;
        end
        
    end
    
end
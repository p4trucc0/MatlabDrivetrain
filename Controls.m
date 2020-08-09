classdef Controls < handle
    % Patrucco, 2020, dummy control class
    % Contains basic controls regarding drivetrain status
    
    properties
        gas_pedal = 0.0; % gas (0 - 1.0)
        brk_pedal = 0.0; % brake (0 - 1.0)
        clc_pedal = 0.0; % clutch (0 - 1.0)
        gear_lever = 0; % gear lever selection
    end
    
    methods
        
        function set_all(obj, n_gas_pedal, n_brk_pedal, n_clc_pedal, n_gear_lever)
            obj.gas_pedal = n_gas_pedal;
            obj.brk_pedal = n_brk_pedal;
            obj.clc_pedal = n_clc_pedal;
            obj.gear_lever = n_gear_lever;
        end
        
        
        
    end
    
    
    
end
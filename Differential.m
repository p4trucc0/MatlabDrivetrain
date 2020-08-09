classdef Differential < handle
% Dummy, simple differential class with a final reduction drive.
% This represents a final differential, with torque going on one axis only.
    
    properties
        final_drive = [];
    end
    
    methods
        
        function obj = Differential(n_final_drive)
            obj.final_drive = n_final_drive;
        end
        
    end
    
    
end
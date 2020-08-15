classdef Axle < handle
    % Now I'm using it to gather information about axle-wheel radius etc.
    
    properties
        wheels = 2;
        J = 0.05;
        radius = .3;
        tol_close2still = 5;
        tol_still = .1;
        close2still = false;
        still = false;
    end
    
    methods
        
        function update_still_situation(obj, w)
            obj.close2still = w <= obj.tol_close2still;
            obj.still = w <= obj.tol_still;
        end
        
    end
    
    
end
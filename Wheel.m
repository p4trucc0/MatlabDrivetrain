classdef Wheel < handle
    
    properties
        m = 10.0;
        R = 0.3;
        J = 0.01;
        pacejka = [];
    end
    
    methods
        function obj = Wheel(n_R, n_J, n_pacejka)
            obj.J = n_J;
            obj.R = n_R;
            obj.pacejka = n_pacejka;
        end
    end
    
    
end
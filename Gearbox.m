classdef Gearbox < handle
    
    properties
        ratios = [];
        efficiencies = [];
        has_reverse = true;
    end
    
    methods
        
        function obj = Gearbox(n_ratios, n_efficiencies, n_has_reverse)
            obj.ratios = n_ratios;
            obj.efficiencies = n_efficiencies;
            obj.has_reverse = n_has_reverse;
        end
        
        % By convention, first element in the vector always represents
        % reverse, even if absent.
        function r = get_ratio(obj, gear)
            if gear == 0
                if obj.has_reverse
                    r = obj.ratios(1);
                else
                    r = NaN;
                end
            else
                r = obj.ratios(gear + 1);
            end
        end
        
        function r = get_efficiency(obj, gear)
            if gear == 0
                if obj.has_reverse
                    r = obj.efficiencies(1);
                else
                    r = NaN;
                end
            else
                r = obj.efficiencies(gear + 1);
            end
        end
        
    end
    
end
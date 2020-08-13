classdef Brake < handle
    
    properties
        f = 0.3; % [], friction coefficient (dynamic)
        R1 = 0.07; % [m], inner radius of the pad
        R2 = 0.11; % [m], outer radius of the pad
        alpha = pi/4; % [rad], extension of the pad
        max_Q = 6000; % maximum appliable force.
        Rm = 0.09;
    end
    
    methods
        
        % Class constructor
        function obj = Brake(n_f, n_R1, n_R2, n_alpha, n_max_Q)
            obj.f = n_f;
            obj.R1 = n_R1;
            obj.R2 = n_R2;
            obj.alpha = n_alpha;
            obj.max_Q = n_max_Q;
            obj.Rm = (obj.R1 + obj.R2) / 2;
        end
        
        % brake force from pressure at the pad.
        function Q = get_normal_brake_force(obj, p)
            Q = obj.alpha * p * obj.Rm * (obj.R2 - obj.R1);
        end
        
        function Mf = get_brake_torque(obj, Q)
            Mf = obj.f * Q * obj.Rm;
        end
        
        function Mf = get_simple_brake_torque(obj, brk_pedal)
            Mf = obj.f * obj.Rm * obj.max_Q * brk_pedal;
        end
        
    end
    
end
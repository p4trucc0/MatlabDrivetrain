classdef BrakeSystem2Axle < handle
    
    properties
        front_brake = [];
        rear_brake = [];
        controls = [];
        distribution = struct('x', [0 1], 'y_rf', [1 .5]);
    end
    
    methods
        
        function obj = BrakeSystem2Axle(n_front_brake, n_rear_brake, n_controls, n_distribution)
            obj.front_brake = n_front_brake;
            obj.rear_brake = n_rear_brake;
            obj.controls = n_controls;
            obj.distribution = n_distribution;
        end
        
        function [Mf, Mr] = get_brake_torques(obj)
            brk_x = obj.controls.brk_pedal;
            brk_f = brk_x;
            brk_r = interp1(obj.distribution.x, obj.distribution.y_rf, brk_x);
            if brk_r > brk_f
                brk_f = (brk_f / brk_r) * brk_x;
                brk_r = brk_x;
            end
            Mf = brk_f.get_simple_brake_torque(brk_f);
            Mr = brk_r.get_simple_brake_torque(brk_r);
        end
        
    end
    
end
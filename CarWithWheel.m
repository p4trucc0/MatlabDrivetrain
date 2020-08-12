classdef CarWithWheel < handle
    
    % Patrucco, 2020
    % Simple car model consisting of a mass and a wheel, with variable
    % force acting on it (to simulate non-tractive axes)
    
    % TODO: add brake object.
    properties
        mass = NaN; %kg
        wheel_inertia = NaN; % kg * m
        wheel_pacejka_param = []; % struct containing wheel pacejka parameters
        tractive_mass_fraction = NaN; % []
        radius = NaN; % m
        front_surface = NaN; % m2
        drag_coeff = NaN; % []
        drivetrain = []; % Drivetrain object
    end
    
    methods
        
        % Constructor
        function obj = CarWithWheel(n_mass, n_wheel_inertia, ...
                n_wheel_pacejka_param, n_tractive_mass_fraction, ...
                n_radius, n_front_surface, n_drag_coeff, n_drivetrain)
            obj.mass = n_mass;
            obj.wheel_inertia = n_wheel_inertia;
            obj.wheel_pacejka_param = n_wheel_pacejka_param;
            obj.tractive_mass_fraction = n_tractive_mass_fraction;
            obj.radius = n_radius;
            obj.front_surface = n_front_surface;
            obj.drag_coeff = n_drag_coeff;
            obj.drivetrain = n_drivetrain;
        end
        
        % Determine acceleration
        % x1v contains: 
        % - engine speed 
        % - wheel (differential) speed
        % - car body speed
        function [x2v, fv] = get_acc(obj, x1v)
            % Determine external forces
            x1_engine = x1v(1); x1_wheel = x1v(2); x1_body = x1v(3);
            F_aer = .5*1.2*obj.front_surface*obj.drag_coeff*(x1_body^2);
            % get slip ratio
            if ((x1_body == 0.0) && (x1_wheel == 0.0))
                k = 0;
            else
                if (x1_body < 1.0)
                    k = 100 * (x1_wheel*obj.radius - x1_body) / (x1_wheel*obj.radius);
                else
                    k = 100 * (x1_wheel*obj.radius - x1_body) / x1_body;
                end
            end
            if k < -100
                %keyboard
            end
            if abs(k) > 100
                k = 20*sign(k);
            end
            [Fx_pac, ~, ~] = pacejka96(obj.wheel_pacejka_param, obj.mass*9.81*obj.tractive_mass_fraction / 1000, 0.0, k, 0.0);
            x2_body = (Fx_pac - F_aer) / obj.mass;
            % apply drivetrain law
            wheel_torque = -Fx_pac * obj.radius;
            [x2_engine, x2_wheel] = obj.drivetrain.get_shaft_acc(x1_wheel, x1_engine, wheel_torque, obj.wheel_inertia);
            x2v = [x2_engine; x2_wheel; x2_body];
            fv = [F_aer Fx_pac wheel_torque];
        end
        
    end
    
    
end
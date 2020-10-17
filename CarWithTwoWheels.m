classdef CarWithTwoWheels < handle
    
    % Patrucco, 2020
    % Simple car model consisting of a mass (statically distributed) and
    % two wheels. Basis for further development.
    
    % TODO: add brake object.
    properties
        mass = NaN; %kg
        wheel_front = []; 
        wheel_rear = [];
        front_mass_fraction = NaN; % []
        front_surface = NaN; % m2
        drag_coeff = NaN; % []
        drivetrain = []; % Drivetrain object
        dt = .01; % integration step.
        tol_speed_static = 0.01;
        min_speed_for_slip = 1.0;
    end
    
    methods
        
        % Constructor
        function obj = CarWithTwoWheels(n_mass, n_wheel_front, ...
                n_wheel_rear, n_front_mass_fraction, ...
                n_front_surface, n_drag_coeff, n_drivetrain)
            obj.mass = n_mass;
            obj.wheel_front = n_wheel_front;
            obj.wheel_rear = n_wheel_rear;
            obj.front_mass_fraction = n_front_mass_fraction;
            obj.front_surface = n_front_surface;
            obj.drag_coeff = n_drag_coeff;
            obj.drivetrain = n_drivetrain;
        end
        
        % Determine acceleration
        % x1v contains: 
        function [x2v, params] = get_acc(obj, x1v)
            % Decompose input vector
            th1_a = x1v(3);
            th1_p = x1v(4);
            x1_b = x1v(5);
            % Evaluate external forces
            k_a = obj.eval_slip(th1_a, obj.drivetrain.wheel_front.R, x1_b);
            k_p = obj.eval_slip(th1_p, obj.drivetrain.wheel_rear.R, x1_b);
            Fz_a = obj.mass*9.81*obj.front_mass_fraction;
            Fz_p = obj.mass*9.81*(1 - obj.front_mass_fraction);
            [Fx_a, ~, ~] = pacejka96(obj.wheel_front.pacejka, Fz_a/1000, 0.0, k_a, 0.0);
            [Fx_p, ~, ~] = pacejka96(obj.wheel_rear.pacejka, Fz_p/1000, 0.0, k_p, 0.0);
            Fx_aer = .5*1.2*obj.front_surface*obj.drag_coeff*(x1_b^2);
            w4dt = x1v(1:4);
            [x2_dt, ap] = obj.drivetrain.get_shaft_acc(w4dt, Fx_a, Fx_p);
            x2_b = (Fx_a + Fx_p - Fx_aer) / obj.mass;
            x2v = [x2_dt; x2_b];
            params = ap;
            params.Fx_a = Fx_a;
            params.Fx_p = Fx_p;
            params.Fz_a = Fz_a;
            params.Fz_p = Fz_p;
        end
        
        function kk = eval_slip(obj, w, r, v)
            if r < obj.min_speed_for_slip
                kk = 100 * (w*r - v) /  obj.min_speed_for_slip;
            else
                kk = 100 * (w*r - v) /  v;
            end
        end
        
    end
    
    
end
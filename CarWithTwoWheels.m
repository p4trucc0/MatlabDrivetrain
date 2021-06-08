classdef CarWithTwoWheels < handle
    
    % Patrucco, 2020
    % Simple car model consisting of a mass (statically distributed) and
    % two wheels. Basis for further development.
    % Edit 28/10/2020 - 19:17: adding dynamic weight redistribution.
    
    % TODO: add brake object.
    properties
        mass = NaN; %kg
        wheel_front = []; 
        wheel_rear = [];
        front_surface = NaN; % m2
        drag_coeff = NaN; % []
        drivetrain = []; % Drivetrain object
        dt = .01; % integration step.
        tol_speed_static = 0.01;
        min_speed_for_slip = 1.0;
        x_front_wheel = 1.3;
        x_rear_wheel = -1.3;
        z_cog = .7;
        lift_coeff = 0.0; % positive, generates a force upwards.
    end
    
    methods
        
        % Constructor
        function obj = CarWithTwoWheels(n_mass, n_wheel_front, ...
                n_wheel_rear, ...
                n_front_surface, n_drag_coeff, n_drivetrain, ...
                n_x_front_wheel, n_x_rear_wheel, n_z_cog, ...
                n_lift_coeff)
            obj.mass = n_mass;
            obj.wheel_front = n_wheel_front;
            obj.wheel_rear = n_wheel_rear;
            obj.front_surface = n_front_surface;
            obj.drag_coeff = n_drag_coeff;
            obj.drivetrain = n_drivetrain;
            obj.x_front_wheel = n_x_front_wheel;
            obj.x_rear_wheel = n_x_rear_wheel;
            obj.z_cog = n_z_cog;
            obj.lift_coeff = n_lift_coeff;
        end
        
        % Determine acceleration
        % x1v current speed, x2v_p previous acceleration.
        function [x2v, params] = get_acc(obj, x1v, x2v_p)
            % Decompose input vector
            th1_a = x1v(3);
            th1_p = x1v(4);
            x1_b = x1v(5);
            x2_b = x2v_p(5); % body acceleration
            % Evaluate external forces
            Fz = obj.mass*9.81 - .5*1.2*obj.front_surface*obj.lift_coeff*(x1_b^2);
            Fz_p = (obj.mass*x2_b*obj.z_cog + Fz*obj.x_front_wheel) / (obj.x_front_wheel + obj.x_rear_wheel);
            Fz_a = (Fz - Fz_p);
            k_a = obj.eval_slip(th1_a, obj.drivetrain.wheel_front.R, x1_b);
            k_p = obj.eval_slip(th1_p, obj.drivetrain.wheel_rear.R, x1_b);
            [Fx_a, ~, ~] = pacejka96(obj.wheel_front.pacejka, Fz_a/2000, 0.0, k_a, 0.0);
            [Fx_p, ~, ~] = pacejka96(obj.wheel_rear.pacejka, Fz_p/2000, 0.0, k_p, 0.0);
            Fx_a = 2*Fx_a;
            Fx_p = 2*Fx_p; % pacejka describes ONE of the two wheels!
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
            if v < obj.min_speed_for_slip
                kk = 100 * (w*r - v) /  obj.min_speed_for_slip;
            else
                kk = 100 * (w*r - v) /  v;
            end
        end
        
    end
    
    
end
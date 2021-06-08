classdef CarWithFourWheels < handle
    
    % Patrucco, 2021
    % Port of C++ version of the same object
    
    properties
        vg = []; % vehicle geometry struct
        wheel_fl = [];
        wheel_fr = [];
        wheel_rl = [];
        wheel_rr = []; % wheel objects.
        drivetrain = [];
        controls = [];
        steering_sensitivity = [];
        min_speed_for_slip = 1.0;
        tol_speed_static = 0.01;
        apply_tyre_forces = true;
    end
    
    methods
        
        % Constructor
        function obj = CarWithFourWheels(n_vg, n_wheel_fl, n_wheel_fr, ...
                n_wheel_rl, n_wheel_rr, n_drivetrain, n_controls, ...
                n_steering_sensitivity)
            obj.vg = n_vg;
            obj.wheel_fl = n_wheel_fl;
            obj.wheel_fr = n_wheel_fr;
            obj.wheel_rl = n_wheel_rl;
            obj.wheel_rr = n_wheel_rr;
            obj.drivetrain = n_drivetrain;
            obj.controls = n_controls;
            obj.steering_sensitivity = n_steering_sensitivity;
        end
        
        function kk = eval_long_slip(obj, w, r, v)
            if (abs(v) < obj.min_speed_for_slip)
                kk = (100.0 * (w*r - v)) / obj.min_speed_for_slip;
            else
                kk = (100.0 * (w*r - v)) / abs(v) ;
            end
        end
        
        function out = eval_lat_slip(obj, ang_wheel, vx, vy)
            SLIPANGLE_ZEROTOL = 0.01;
            if ((abs(vx) < SLIPANGLE_ZEROTOL) && (abs(vy) < SLIPANGLE_ZEROTOL))
                out = 0.0;
            else
                if (abs(vx) < SLIPANGLE_ZEROTOL)
                    if (vy > 0.0)
                        atyx = pi/2.0;
                    else
                        atyx = -pi/2.0;
                    end
                else
                    if (vx > 0.0)
                        atyx = atan(vy/vx);
                    else
                        atyx = -atan(vy/vx);
                    end
                end
                out = (ang_wheel - atyx);
            end
        end
        
        % x0v: positions
        % x1v: speeds
        % x2v_p: accelerations in previous time frame
        % x2v: accelerations
        % dap: additional parameters.
        function [x2v, dap] = get_acc(obj, x0v, x1v, x2v_p)
            % Should decide if just copy-paste from edm_car or modify that
            % function to make it more flexible.
        end
        
    end
    
    
end
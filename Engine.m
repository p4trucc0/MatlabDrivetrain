classdef Engine < handle
    % Patrucco, 2020
    % Engine class to be fed with a torque map and inertial data, for full
    % vehicle drivetrain simulation. Ideally, father of both NAEngine and
    % TurboEngine
    
    properties
        lambda_v = []; % lambda values corresponding to rows in the matrix
        rpm_v = []; % rpm values corresponding to matrix columns
        torque_map = []; % matrix numel(lambda_v) x numel(rpm_v)
        fv_0 = 0.0; % friction coefficient (proportional to speed)
        fv_1 = 0.0; % fr. coeff. (proportional to speed, squared)
        Jm = NaN; % Inertia of engine and flywheel
    end
    
    methods
        
        % Class constructor
        function obj = Engine(n_lambda_v, n_rpm_v, n_torque_map, n_fv_0, n_fv_1, n_Jm)
            obj.lambda_v = n_lambda_v;
            obj.rpm_v = n_rpm_v;
            obj.torque_map = n_torque_map;
            obj.fv_0 = n_fv_0;
            obj.fv_1 = n_fv_1;
            obj.Jm = n_Jm;
        end
        
        % Maximum possible torque for current lambda and rpm.
        % This has already been done in C++, thorough emulation is not
        % needed
        function tq = get_max_torque(obj, l, w)
            lambda_ind = interp1(obj.lambda_v, 1:numel(obj.lambda_v), l);
            rpm_ind = interp1(obj.rpm_v, 1:numel(obj.rpm_v), w);
            % tq = interp2(obj.torque_map, lambda_ind, rpm_ind);
            tq = interp2(obj.torque_map, rpm_ind, lambda_ind);
        end
        
        % TODO: introduce function taking friction into account.
        function tq = get_torque(obj, l, w)
            tq_max = obj.get_max_torque(l, w);
            tq = tq_max - (obj.fv_0 * w) - (obj.fv_1 * (w^2));
            if isnan(tq)
                tq = 0;
            end
        end
        
    end
    
end
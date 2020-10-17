classdef cardyn_scheme_2d < handle
    % Graphic object to show car data (i.e. a car with wheels and some
    % other things like arrows to show forces etc.). Units in mm
    
    properties
        parent = [];
        ax = [];
        ax_background_color = [0 0 0];
        ax_car_color = [1 0 0];
        ax_wheels_color = [1 1 1];
        ax_writings_color = [0 1 0];
        ax_bglines_color = [1 1 0];
        car_lines = [];
        car_silhouette = struct('x', [-2200, 2200], ...
            'z', [0, 0]);
        car_origin = [0 200];
        car_position = [0 0];
        wheel_radius = 300;
        wheel_front_x = 1400;
        wheel_rear_x = -1500;
        wheel_front_z = 0;
        wheel_rear_z = 0;
        wheel_front_th = 0;
        wheel_rear_th = 0;
        wheel_front_lines = [];
        wheel_rear_lines = [];
        circle_pts = 100;
    end
    
    methods
        
        function obj = cardyn_scheme_2d(n_parent)
            obj.parent = n_parent;
            obj.ax = axes('Parent', obj.parent);
            obj.ax.PlotBoxAspectRatio = [1 1 1];
            obj.ax.XAxis.Visible = 'off';
            obj.ax.YAxis.Visible = 'off';
            obj.ax.Color = obj.ax_background_color;
            obj.generate_car_lines();
            obj.generate_wheels_lines();
            obj.ax.PlotBoxAspectRatio = [1 1 1];
            obj.ax.XLim = [-3000 3000];
            obj.ax.YLim = [-3000 3000];
            keyboard;
        end
        
        function generate_car_lines(obj)
            for ii = 1:length(obj.car_silhouette.x)-1
                obj.car_lines = [obj.car_lines, ...
                    line([obj.car_silhouette.x(ii), obj.car_silhouette.x(ii+1)], ...
                    [obj.car_silhouette.z(ii), obj.car_silhouette.z(ii+1)], ...
                    'Parent', obj.ax, 'Color', obj.ax_car_color, 'LineWidth', 2)];
            end
        end
        
        % TODO: add car and wheel roto-translation
        
        function generate_wheels_lines(obj)
            ang = linspace(0, 2*pi, obj.circle_pts);
            for ii = 1:obj.circle_pts-1
                obj.wheel_front_lines = [obj.wheel_front_lines, ...
                    line([obj.wheel_front_x+obj.wheel_radius*cos(ang(ii)), obj.wheel_front_x+obj.wheel_radius*cos(ang(ii+1))], ...
                    [obj.wheel_front_z+obj.wheel_radius*sin(ang(ii)), obj.wheel_front_z+obj.wheel_radius*sin(ang(ii+1))], ...
                    'Parent', obj.ax, 'Color', obj.ax_wheels_color, 'LineWidth', 2)];
                obj.wheel_rear_lines = [obj.wheel_rear_lines, ...
                    line([obj.wheel_rear_x+obj.wheel_radius*cos(ang(ii)), obj.wheel_rear_x+obj.wheel_radius*cos(ang(ii+1))], ...
                    [obj.wheel_rear_z+obj.wheel_radius*sin(ang(ii)), obj.wheel_rear_z+obj.wheel_radius*sin(ang(ii+1))], ...
                    'Parent', obj.ax, 'Color', obj.ax_wheels_color, 'LineWidth', 2)];
            end
        end
        
    end
    
    
    
end
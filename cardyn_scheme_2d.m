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
        ax_writings_fontsize = 14;
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
        Fx_front = 0;
        Fx_rear = 0;
        Fz_front = 0;
        Fz_rear = 0;
        Fx_front_text = [];
        Fx_rear_text = [];
        Fz_front_text = [];
        Fz_rear_text = [];
        ax_limit = 3000;
        N_bg_lines = 2;
        bg_lines = [];
    end
    
    methods
        
        function obj = cardyn_scheme_2d(n_parent)
            obj.parent = n_parent;
            obj.ax = axes('Parent', obj.parent);
            obj.ax.PlotBoxAspectRatio = [1 1 1];
            obj.ax.XAxis.Visible = 'off';
            obj.ax.YAxis.Visible = 'off';
            obj.ax.Color = obj.ax_background_color;
            obj.generate_bg_lines();
            obj.generate_car_lines();
            obj.generate_wheels_lines();
            obj.generate_texts();
            obj.ax.PlotBoxAspectRatio = [1 1 1];
            obj.ax.XLim = [-obj.ax_limit obj.ax_limit];
            obj.ax.YLim = [-obj.ax_limit obj.ax_limit];
            %keyboard;
        end
        
        function update_all(obj)
            % TODO: roto-traslation of car silhouette
            obj.wheel_front_lines(end).XData = [obj.wheel_front_x, obj.wheel_front_x + obj.wheel_radius*cos(-obj.wheel_front_th)];
            obj.wheel_front_lines(end).YData = [obj.wheel_front_z, obj.wheel_front_z + obj.wheel_radius*sin(-obj.wheel_front_th)];
            obj.wheel_rear_lines(end).XData = [obj.wheel_rear_x, obj.wheel_rear_x + obj.wheel_radius*cos(-obj.wheel_rear_th)];
            obj.wheel_rear_lines(end).YData = [obj.wheel_rear_z, obj.wheel_rear_z + obj.wheel_radius*sin(-obj.wheel_rear_th)];
            xv = linspace(-obj.ax_limit, obj.ax_limit, obj.N_bg_lines + 1);
            for ii = 1:obj.N_bg_lines
                % obj.bg_lines(ii).XData = obj.bg_lines(ii).XData - obj.car_position(1);
                obj.bg_lines(ii).XData = (xv(ii) - obj.car_position(1))*ones(1, 2);
                while (obj.bg_lines(ii).XData(1) < -obj.ax_limit)
                    obj.bg_lines(ii).XData = obj.bg_lines(ii).XData + 2*obj.ax_limit;
                end
                while (obj.bg_lines(ii).XData(1) > +obj.ax_limit)
                    obj.bg_lines(ii).XData = obj.bg_lines(ii).XData - 2*obj.ax_limit;
                end
            end
            obj.Fx_front_text.String = num2str(round(obj.Fx_front));
            obj.Fz_front_text.String = num2str(round(obj.Fz_front));
            obj.Fx_rear_text.String = num2str(round(obj.Fx_rear));
            obj.Fz_rear_text.String = num2str(round(obj.Fz_rear));
        end
        
        function generate_bg_lines(obj)
            xv = linspace(-obj.ax_limit, obj.ax_limit, obj.N_bg_lines + 1);
            for ii = 1:obj.N_bg_lines
                obj.bg_lines = [obj.bg_lines, ...
                    line([xv(ii) xv(ii)], [-obj.ax_limit, obj.ax_limit], ...
                    'Parent', obj.ax, 'Color', obj.ax_bglines_color, ...
                    'LineWidth', 1)];
            end
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
            obj.wheel_front_lines = [obj.wheel_front_lines, ...
                line([obj.wheel_front_x, obj.wheel_front_x + obj.wheel_radius*cos(obj.wheel_front_th)], ...
                [obj.wheel_front_z, obj.wheel_front_z + obj.wheel_radius*sin(obj.wheel_front_th)], ...
                'Parent', obj.ax, 'Color', obj.ax_wheels_color, 'LineWidth', 2)];
            obj.wheel_rear_lines = [obj.wheel_rear_lines, ...
                line([obj.wheel_rear_x, obj.wheel_rear_x + obj.wheel_radius*cos(obj.wheel_rear_th)], ...
                [obj.wheel_rear_z, obj.wheel_rear_z + obj.wheel_radius*sin(obj.wheel_rear_th)], ...
                'Parent', obj.ax, 'Color', obj.ax_wheels_color, 'LineWidth', 2)];
        end
        
        function generate_texts(obj)
            obj.Fx_front_text = text(obj.wheel_front_x, -1000, num2str(round(obj.Fx_front)), ...
                'Parent', obj.ax, 'Color', obj.ax_writings_color, 'FontSize', obj.ax_writings_fontsize);
            obj.Fz_front_text = text(obj.wheel_front_x, -500, num2str(round(obj.Fz_front)), ...
                'Parent', obj.ax, 'Color', obj.ax_writings_color, 'FontSize', obj.ax_writings_fontsize);
            obj.Fx_rear_text = text(obj.wheel_rear_x, -1000, num2str(round(obj.Fx_rear)), ...
                'Parent', obj.ax, 'Color', obj.ax_writings_color, 'FontSize', obj.ax_writings_fontsize);
            obj.Fz_rear_text = text(obj.wheel_rear_x, -500, num2str(round(obj.Fz_rear)), ...
                'Parent', obj.ax, 'Color', obj.ax_writings_color, 'FontSize', obj.ax_writings_fontsize);
        end
        
    end
    
    
    
end
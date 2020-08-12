classdef GaugeObj < handle
    % Patrucco, 2020
    % Graphic object containing a descriptive string, a slider (used as a
    % needle) and a numerical description. As of now, only passive, but it
    % would be interesting to use it as a control, too.
    
    properties
        panel = []; % parent panel
        name = ''; % Name of property to be displayed
        pos = []; % position into parent panel
        val = 0.0; % value
        minVal = 0.0;
        maxVal = 0.0;
        descr_control = [];
        slider_control = [];
        val_control = [];
    end
    
    methods
        
        function obj = GaugeObj(n_panel, n_name, n_pos, n_val, n_minVal, n_maxVal)
            obj.panel = n_panel;
            obj.name = n_name;
            obj.pos = n_pos;
            obj.val = n_val;
            obj.minVal = n_minVal;
            obj.maxVal = n_maxVal;
            obj.descr_control = uicontrol('parent', obj.panel, 'Style','edit', ...
                'units', 'norm', 'pos', ...
                [obj.pos(1) obj.pos(2) obj.pos(3)*.2 obj.pos(4)], ...
                'BackgroundColor', [.85 .85 .85], ...
                'String', obj.name, 'FontSize', 9, 'enable', 'inactive');
            obj.val_control = uicontrol('parent', obj.panel, 'Style','edit', ...
                'units', 'norm', 'pos', ...
                [(obj.pos(1) + (obj.pos(3) - obj.pos(1))*.8) obj.pos(2) obj.pos(3)*.2 obj.pos(4)], ...
                'BackgroundColor', [1 1 1], ...
                'String', num2str(obj.val), 'FontSize', 9, 'enable', 'inactive');
            obj.slider_control = uicontrol('parent', obj.panel, 'Style', 'slider', ...
                'units', 'norm', 'pos', ...
                [(obj.pos(1) + (obj.pos(3) - obj.pos(1))*.2) obj.pos(2) obj.pos(3)*.6 obj.pos(4)], ...
                'enable', 'inactive', 'Value', ((obj.val - obj.minVal)/(obj.maxVal - obj.minVal)*1));
        end
        
        function set_val(obj, n_val)
            obj.val = n_val;
            obj.val_control.String = num2str(n_val);
            obj.slider_control.Value = ((obj.val - obj.minVal)/(obj.maxVal - obj.minVal)*1);
        end
        
    end
    
end
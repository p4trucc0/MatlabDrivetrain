function DriveAnalyzer_v0()
% Simple GUI to analyze log data coming in tabular form.


MAIN_BG_COLOR = [1 1 1];
PANEL_BG_COLOR = [.85 .85 .85];
HSEP_0 = .4;
HSEP_1 = .4;
VSEP_0 = .2;
VSEP_1 = .8;
AXIS_H_OFFSET = .02;
AXIS_V_OFFSET = .1;

% Globals
AvailableFields = {};
Data = [];
SelectedX = '';
SelectedY1 = {};
SelectedY2 = {};

Gui = figure('position', [50 50 1200 600]);
MainFrame = uipanel('parent', Gui, 'units', 'norm', 'pos',[0 0 1 1], ...
        'BorderType','None', 'BackgroundColor', MAIN_BG_COLOR);
HFrame0 = uipanel('parent', MainFrame, 'units', 'norm', 'pos',[0, 1-HSEP_0, 1, HSEP_0], ...
        'BorderType','None', 'BackgroundColor', PANEL_BG_COLOR);
HFrame1 = uipanel('parent', MainFrame, 'units', 'norm', 'pos',[0, 1-(HSEP_0+HSEP_1), 1, HSEP_1], ...
        'BorderType','None', 'BackgroundColor', PANEL_BG_COLOR);
HFrame2 = uipanel('parent', MainFrame, 'units', 'norm', 'pos',[0, 0, 1, 1-(HSEP_0+HSEP_1)], ...
        'BorderType','None', 'BackgroundColor', PANEL_BG_COLOR);

Axis1 = axes('parent', HFrame0, 'units', 'norm', 'pos', [VSEP_0 + AXIS_H_OFFSET, AXIS_V_OFFSET, (1 - VSEP_0 - 2*AXIS_H_OFFSET), 1 - 2*AXIS_V_OFFSET]); grid(Axis1, 'on');
Axis2 = axes('parent', HFrame1, 'units', 'norm', 'pos', [VSEP_0 + AXIS_H_OFFSET, AXIS_V_OFFSET, (1 - VSEP_0 - 2*AXIS_H_OFFSET), 1 - 2*AXIS_V_OFFSET]); grid(Axis2, 'on');
linkaxes([Axis1, Axis2], 'x');
lines1 = [line('parent', Axis1, 'XData', [], 'YData', [], 'Color', 'r', 'Marker', 'none', 'LineStyle', '-', 'LineWidth', 2), ...
    line('parent', Axis1, 'XData', [], 'YData', [], 'Color', 'b', 'Marker', 'none', 'LineStyle', '-', 'LineWidth', 2), ...
    line('parent', Axis1, 'XData', [], 'YData', [], 'Color', 'm', 'Marker', 'none', 'LineStyle', '-', 'LineWidth', 2), ...
    line('parent', Axis1, 'XData', [], 'YData', [], 'Color', 'c', 'Marker', 'none', 'LineStyle', '-', 'LineWidth', 2), ...
    line('parent', Axis1, 'XData', [], 'YData', [], 'Color', 'k', 'Marker', 'none', 'LineStyle', '--', 'LineWidth', 1)];
lines2 = [line('parent', Axis2, 'XData', [], 'YData', [], 'Color', 'r', 'Marker', 'none', 'LineStyle', '-', 'LineWidth', 2), ...
    line('parent', Axis2, 'XData', [], 'YData', [], 'Color', 'b', 'Marker', 'none', 'LineStyle', '-', 'LineWidth', 2), ...
    line('parent', Axis2, 'XData', [], 'YData', [], 'Color', 'm', 'Marker', 'none', 'LineStyle', '-', 'LineWidth', 2), ...
    line('parent', Axis2, 'XData', [], 'YData', [], 'Color', 'c', 'Marker', 'none', 'LineStyle', '-', 'LineWidth', 2), ...
    line('parent', Axis2, 'XData', [], 'YData', [], 'Color', 'k', 'Marker', 'none', 'LineStyle', '--', 'LineWidth', 1)];

List1 = uicontrol('parent', HFrame0, 'style', 'listbox', ...
    'units', 'norm', 'position', [AXIS_H_OFFSET, AXIS_V_OFFSET, VSEP_0-2*AXIS_H_OFFSET, 1-2*AXIS_V_OFFSET], ...
    'string',AvailableFields, 'Tag', 'List1', 'Callback',@ListChanged, 'max', 5);
List2 = uicontrol('parent', HFrame1, 'style', 'listbox', ...
    'units', 'norm', 'position', [AXIS_H_OFFSET, AXIS_V_OFFSET, VSEP_0-2*AXIS_H_OFFSET, 1-2*AXIS_V_OFFSET], ...
    'string',AvailableFields, 'Tag', 'List2', 'Callback',@ListChanged, 'max', 5);
ListX = uicontrol('parent', HFrame2, 'style', 'listbox', ...
    'units', 'norm', 'position', [VSEP_1 + AXIS_H_OFFSET, AXIS_V_OFFSET, 1-VSEP_1-2*AXIS_H_OFFSET, 1-2*AXIS_V_OFFSET], ...
    'string',AvailableFields, 'Tag', 'ListX', 'Callback',@ListChanged, 'max', 1);

LoadBtn = uicontrol('parent', HFrame2, 'style','pushbutton', ...
    'units','norm', 'position', [AXIS_H_OFFSET, AXIS_V_OFFSET, (VSEP_0-2*AXIS_H_OFFSET), 0.3], 'String', 'Open', ...
    'FontName', 'Arial', 'FontSize', 12, 'Callback', @LoadFcn, ...
    'BackgroundColor', [.9 .9 .9], 'Tag', 'Load');

%keyboard
    function LoadFcn(obj, event)
        [fname, ffolder] = uigetfile('*.txt');
        Data = table2struct(readtable([ffolder, filesep, fname]), 'ToScalar', 1);
        AvailableFields = fieldnames(Data);
        ind2excl = [];
        for i_af = 1:length(AvailableFields)
            if ~isnumeric(Data.(AvailableFields{i_af}))
                ind2excl = [ind2excl;  i_af];
            end
        end
        AvailableFields(ind2excl) = [];
        set(List1, 'string', AvailableFields);
        set(List2, 'string', AvailableFields);
        set(ListX, 'string', AvailableFields);
    end

    function ListChanged(obj, event)
        t = get(obj, 'Tag');
        SelectedX = AvailableFields{ListX.Value};
        SelectedY1 = AvailableFields(List1.Value);
        SelectedY2 = AvailableFields(List2.Value);
        for i_line = 1:length(lines1)
            if i_line <= length(SelectedY1)
            set(lines1(i_line), 'XData', Data.(SelectedX));
            set(lines1(i_line), 'YData', Data.(SelectedY1{i_line}));
            else
            set(lines1(i_line), 'XData', []);
            set(lines1(i_line), 'YData', []);
            end
        end
        for i_line = 1:length(lines2)
            if i_line <= length(SelectedY2)
            set(lines2(i_line), 'XData', Data.(SelectedX));
            set(lines2(i_line), 'YData', Data.(SelectedY2{i_line}));
            else
            set(lines2(i_line), 'XData', []);
            set(lines2(i_line), 'YData', []);
            end
        end
        set(Axis1, 'XLim', [min(Data.(SelectedX)) max(Data.(SelectedX))]);
    end

    


end
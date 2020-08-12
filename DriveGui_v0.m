function DriveGui_v0()

% Cosmetic/Graphic constants
MainBackgroundColor = [1 1 1];
GaugeBackgroundColor = [1 1 1];
PanelBackgroundColor = [.85 .85 .85];
Hsep1 = .5;
Vsep1 = 1/3;
Vsep2 = 2/3;
SmallFont = 10;

% Functional Constants
OperatingMode = 1; % 1: automation; 2: live (joypad)

% Global objects (such as drivetrain, etc)
% TODO: define function to load and save a car's properties
Car = generate_dummy_car();


% Graphical build.
Gui = figure('position', [50 50 1200 600]);
MainFrame = uipanel('parent', Gui, 'units', 'norm', 'pos',[0 0 1 1], ...
        'BorderType','None', 'BackgroundColor', MainBackgroundColor);

% Right Panel, containing control show.
RightPanel = uipanel('parent', MainFrame, 'units', 'norm', 'pos', [Hsep1 0 ...
    (1 - Hsep1) 1], 'BackgroundColor', PanelBackgroundColor);

% SpeedPanel - Contains RPM and speed indicators.
SpeedPanel = uipanel('parent', RightPanel, 'units', 'norm', 'pos', [0 Vsep2 ...
    1 (1 - Vsep2)], 'BackgroundColor', PanelBackgroundColor);
SpeedPanel_Speed_Body_Descr = uicontrol('parent', SpeedPanel, 'Style','edit', ...
    'units', 'norm', 'pos', [0 2/3 0.2 1/3], 'BackgroundColor', PanelBackgroundColor, ...
    'String', 'V Car [km/h]', 'FontSize', SmallFont, 'enable', 'inactive');
SpeedPanel_Speed_Front_Descr = uicontrol('parent', SpeedPanel, 'Style','edit', ...
    'units', 'norm', 'pos', [0 1/3 0.2 1/3], 'BackgroundColor', PanelBackgroundColor, ...
    'String', 'V Front [km/h]', 'FontSize', SmallFont, 'enable', 'inactive');
SpeedPanel_Speed_Rear_Descr = uicontrol('parent', SpeedPanel, 'Style','edit', ...
    'units', 'norm', 'pos', [0 0/3 0.2 1/3], 'BackgroundColor', PanelBackgroundColor, ...
    'String', 'V Rear [km/h]', 'FontSize', SmallFont, 'enable', 'inactive');
SpeedPanel_Speed_Body_Val = uicontrol('parent', SpeedPanel, 'Style','edit', ...
    'units', 'norm', 'pos', [0.2 2/3 0.2 1/3], 'BackgroundColor', GaugeBackgroundColor, ...
    'String', '0.0', 'FontSize', SmallFont, 'enable', 'inactive');
SpeedPanel_Speed_Front_Val = uicontrol('parent', SpeedPanel, 'Style','edit', ...
    'units', 'norm', 'pos', [0.2 1/3 0.2 1/3], 'BackgroundColor', GaugeBackgroundColor, ...
    'String', '0.0', 'FontSize', SmallFont, 'enable', 'inactive');
SpeedPanel_Speed_Rear_Val = uicontrol('parent', SpeedPanel, 'Style','edit', ...
    'units', 'norm', 'pos', [0.2 0/3 0.2 1/3], 'BackgroundColor', GaugeBackgroundColor, ...
    'String', '0.0', 'FontSize', SmallFont, 'enable', 'inactive');
% Engine speed: this will be changed once a better graphic widget is
% available.
SpeedPanel_Speed_Engine_Descr = uicontrol('parent', SpeedPanel, 'Style','edit', ...
    'units', 'norm', 'pos', [0.4 .5 0.6 .5], 'BackgroundColor', PanelBackgroundColor, ...
    'String', 'Engine Speed [RPM]', 'FontSize', SmallFont, 'enable', 'inactive');
SpeedPanel_Speed_Engine_Val = uicontrol('parent', SpeedPanel, 'Style','edit', ...
    'units', 'norm', 'pos', [0.4 0 0.6 .5], 'BackgroundColor', GaugeBackgroundColor, ...
    'String', '0', 'FontSize', SmallFont, 'enable', 'inactive');

% CtrlPanel - Contains indicators for the current state of controls
CtrlPanel = uipanel('parent', RightPanel, 'units', 'norm', 'pos', [0 Vsep1 ...
    1 (Vsep2 - Vsep1)], 'BackgroundColor', PanelBackgroundColor);
CtrlPanel_Clc = GaugeObj(CtrlPanel, 'Clutch', [0 .75 1 .25], 0, 0, 1);
CtrlPanel_Acc = GaugeObj(CtrlPanel, 'Gas', [0 .5 1 .25], 1, 0, 1);
CtrlPanel_Brk = GaugeObj(CtrlPanel, 'Brake', [0 .25 1 .25], 0, 0, 1);
CtrlPanel_Gear = GaugeObj(CtrlPanel, 'Gear', [0 0 1 .25], 1, 0, 6);
% CtrlPanel_Clutch.set_val(.75);


% SettingsPanel - Contains buttons and functions for setting up the
% simulation
SettingsPanel = uipanel('parent', RightPanel, 'units', 'norm', 'pos', ...
    [0 0 1 Vsep1], 'BackgroundColor', PanelBackgroundColor);
SettingsPanel_mode_list = uicontrol('parent', SettingsPanel, 'Style', 'listbox', ...
    'units', 'norm', 'pos', [0 0 1/3 1], ...
    'String', {'Automation'; 'Active'}, 'Callback', @change_op_mode);

    function change_op_mode(obj, event)
        OperatingMode = get(obj, 'Value');
    end

end
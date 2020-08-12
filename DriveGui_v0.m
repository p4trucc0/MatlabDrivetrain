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
CurrentVals = struct('w_engine', rpm2rads(4000), 'v_body', 0, ...
    'w_wheel', 0);
AutomationTrack = load('dummy_auto1.mat');
SimulationDeltaTime = .01; %s
DrawDeltaTime = 1/10; %s
SimTime = 0.0;
LastDrawTime = now;
LastDrawSimTime = -1;

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
    'String', num2str(3.6*CurrentVals.v_body), 'FontSize', SmallFont, 'enable', 'inactive');
SpeedPanel_Speed_Front_Val = uicontrol('parent', SpeedPanel, 'Style','edit', ...
    'units', 'norm', 'pos', [0.2 1/3 0.2 1/3], 'BackgroundColor', GaugeBackgroundColor, ...
    'String', num2str(3.6*CurrentVals.w_wheel*Car.radius), 'FontSize', SmallFont, 'enable', 'inactive');
SpeedPanel_Speed_Rear_Val = uicontrol('parent', SpeedPanel, 'Style','edit', ...
    'units', 'norm', 'pos', [0.2 0/3 0.2 1/3], 'BackgroundColor', GaugeBackgroundColor, ...
    'String', '0.0', 'FontSize', SmallFont, 'enable', 'inactive');
% Engine speed: this will be changed once a better graphic widget is
% % available.
% SpeedPanel_Speed_Engine_Descr = uicontrol('parent', SpeedPanel, 'Style','edit', ...
%     'units', 'norm', 'pos', [0.4 .5 0.6 .5], 'BackgroundColor', PanelBackgroundColor, ...
%     'String', 'Engine Speed [RPM]', 'FontSize', SmallFont, 'enable', 'inactive');
% SpeedPanel_Speed_Engine_Val = uicontrol('parent', SpeedPanel, 'Style','edit', ...
%     'units', 'norm', 'pos', [0.4 0 0.6 .5], 'BackgroundColor', GaugeBackgroundColor, ...
%     'String', num2str(rads2rpm(CurrentVals.w_engine)), 'FontSize', SmallFont, 'enable', 'inactive');
SpeedPanel_Engine = GaugeObj(SpeedPanel, 'RPM', [0.4 0 .6 1], rads2rpm(CurrentVals.w_engine), 0, 7100);

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
SettingsPanel_start_button = uicontrol('parent', SettingsPanel, 'style','pushbutton', ...
    'units','norm', 'position', [2/3 0 1/3 1/4], 'String', 'Start', ...
    'FontSize', SmallFont, 'Callback', @StartButtonCbck, ...
    'BackgroundColor', [.9 .9 .9]);


    function change_op_mode(obj, event)
        OperatingMode = get(obj, 'Value');
    end

    % Main loop
    function StartButtonCbck(obj, event)
        switch (OperatingMode)
            case 1
                simulateFromAutomation();
            case 2
                simulateFromJoystick();
        end
    end

    function simulateFromAutomation()
        if ~isempty(AutomationTrack)
            tf = AutomationTrack.autom.t(end);
            LastDrawTime = now;
            while (SimTime < tf)
                acc_p = interp1(AutomationTrack.autom.t, AutomationTrack.autom.gas, SimTime, 'previous');
                brk_p = interp1(AutomationTrack.autom.t, AutomationTrack.autom.brk, SimTime, 'previous');
                clc_p = interp1(AutomationTrack.autom.t, AutomationTrack.autom.clc, SimTime, 'previous');
                gear_p = interp1(AutomationTrack.autom.t, AutomationTrack.autom.gear, SimTime, 'previous');
                Car.drivetrain.controls.set_all(acc_p, brk_p, clc_p, gear_p);
                [av, ~] = Car.get_acc([CurrentVals.w_engine, CurrentVals.w_wheel, ...
                    CurrentVals.v_body]);
                % If writing, do it here.
                CurrentVals.w_engine = CurrentVals.w_engine + av(1)*SimulationDeltaTime;
                CurrentVals.w_wheel = CurrentVals.w_wheel + av(2)*SimulationDeltaTime;
                CurrentVals.v_body = CurrentVals.v_body + av(3)*SimulationDeltaTime;
                SimTime = SimTime + SimulationDeltaTime;
                if (SimTime - LastDrawSimTime > DrawDeltaTime)
                    while (now - LastDrawTime < DrawDeltaTime/(24*3600))
                        pause(.001);
                    end
                    updateView();
                    drawnow;
                    fprintf('%f\n', (now - LastDrawTime)*24*3600);
                    LastDrawTime = LastDrawTime + DrawDeltaTime/(24*3600);
                    LastDrawSimTime = SimTime;
                end
            end
        else
            error('No automation loaded');
        end
    end

    function simulateFromJoystick
        j = HebiJoystick(1);
        exitCondition = 0;
        LastDrawTime = now;
        IntermediateSteps = round(DrawDeltaTime / SimulationDeltaTime); % how many intermediate simulation steps.
        while exitCondition == 0
            b = j.button;
            ax = j.read;
            if b(1) == 1 % first button pressed
                exitCondition = 1;
                break;
            end
            clc_p = abs(ax(2)); % use left handle as clutch.
            if ax(3) < .05
                acc_p = abs(ax(3));
                brk_p = 0;
            else
                brk_p = abs(ax(3));
                acc_p = 0;
            end
            gear_p = 1; % for now, stuck in first gear.
            Car.drivetrain.controls.set_all(acc_p, brk_p, clc_p, gear_p);
            for i_step = 1:IntermediateSteps % perform a chunk of integrations.
                [av, ~] = Car.get_acc([CurrentVals.w_engine, CurrentVals.w_wheel, ...
                    CurrentVals.v_body]);
                % If writing, do it here.
                CurrentVals.w_engine = CurrentVals.w_engine + av(1)*SimulationDeltaTime;
                CurrentVals.w_wheel = CurrentVals.w_wheel + av(2)*SimulationDeltaTime;
                CurrentVals.v_body = CurrentVals.v_body + av(3)*SimulationDeltaTime;
                SimTime = SimTime + SimulationDeltaTime;
            end
            while (now - LastDrawTime < DrawDeltaTime/(24*3600))
                pause(.001);
            end
            updateView();
            drawnow;
            fprintf('%f\n', (now - LastDrawTime)*24*3600);
            LastDrawTime = LastDrawTime + DrawDeltaTime/(24*3600);
            %LastDrawSimTime = SimTime;
        end
    end

    function updateView()
        SpeedPanel_Speed_Body_Val.String = num2str(3.6*CurrentVals.v_body);
        SpeedPanel_Speed_Front_Val.String = num2str(3.6*CurrentVals.w_wheel*Car.radius);
        % SpeedPanel_Speed_Engine_Val.String = num2str(rads2rpm(CurrentVals.w_engine));
        SpeedPanel_Engine.set_val(rads2rpm(CurrentVals.w_engine));
        CtrlPanel_Acc.set_val(Car.drivetrain.controls.gas_pedal);
        CtrlPanel_Brk.set_val(Car.drivetrain.controls.brk_pedal);
        CtrlPanel_Clc.set_val(Car.drivetrain.controls.clc_pedal);
        CtrlPanel_Gear.set_val(Car.drivetrain.controls.gear_lever);
    end


end
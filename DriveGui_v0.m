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
    'w_wheel_front', 0, 'w_wheel_rear', 0, 'Fx_front', 0, ...
    'Fx_rear', 0, 'Fz_front', 0, 'Fz_rear', 0, 'w_clutch', 0, 'x_body', 0, ...
    'th_wheel_front', 0, 'th_wheel_rear', 0);
BufferedVals = CurrentVals;
BufferedVals.N = 0;
ValsToShow = CurrentVals;
AutomationTrack = load('dummy_auto1.mat');
SimulationDeltaTime = .01; %s
DrawDeltaTime = 1/60; %s
SimTime = 0.0;
LastDrawTime = now;
LastDrawSimTime = -1;
OutputFileName = ['debug_', datestr(now, 'yyyy_mm_dd_HH_MM_SS'), '.txt'];

% Global objects (such as drivetrain, etc)
% TODO: define function to load and save a car's properties
Car = generate_dummy_car_2ax();
Car.dt = SimulationDeltaTime;


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
    'String', num2str(3.6*CurrentVals.w_wheel_front*Car.wheel_front.R), 'FontSize', SmallFont, 'enable', 'inactive');
SpeedPanel_Speed_Rear_Val = uicontrol('parent', SpeedPanel, 'Style','edit', ...
    'units', 'norm', 'pos', [0.2 0/3 0.2 1/3], 'BackgroundColor', GaugeBackgroundColor, ...
    'String', num2str(3.6*CurrentVals.w_wheel_rear*Car.wheel_rear.R), 'FontSize', SmallFont, 'enable', 'inactive');
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


% Left Panel, containing graph/scheme
LeftPanel = uipanel('parent', MainFrame, 'units', 'norm', 'pos', [0 0 ...
    Hsep1 1], 'BackgroundColor', PanelBackgroundColor);
CarView = cardyn_scheme_2d(LeftPanel);
%keyboard

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
                cvals = [CurrentVals.w_engine; CurrentVals.w_clutch; ...
                    CurrentVals.w_wheel_front; CurrentVals.w_wheel_rear; ...
                    CurrentVals.v_body];
                [av, a_params] = Car.get_acc(cvals);
                avi = integrate_cgs(cvals, av, SimulationDeltaTime);
                % If writing, do it here.
                CurrentVals.w_engine = avi(1);
                CurrentVals.w_clutch = avi(2);
                CurrentVals.w_wheel_front = avi(3);
                CurrentVals.w_wheel_rear = avi(4);
                CurrentVals.v_body = avi(5);
                CurrentVals.x_body = CurrentVals.x_body + SimulationDeltaTime*CurrentVals.v_body;
                CurrentVals.th_wheel_front = CurrentVals.th_wheel_front + SimulationDeltaTime*CurrentVals.w_wheel_front;
                CurrentVals.th_wheel_rear = CurrentVals.th_wheel_rear + SimulationDeltaTime*CurrentVals.w_wheel_rear;
                CurrentVals.Fx_front = a_params.Fx_a;
                CurrentVals.Fz_front = a_params.Fz_a;
                CurrentVals.Fx_rear = a_params.Fx_p;
                CurrentVals.Fz_rear = a_params.Fz_p;
                SimTime = SimTime + SimulationDeltaTime;
                BufferedVals.w_engine = BufferedVals.w_engine + CurrentVals.w_engine;
                BufferedVals.w_clutch = BufferedVals.w_clutch + CurrentVals.w_clutch;
                BufferedVals.w_wheel_front = BufferedVals.w_wheel_front + CurrentVals.w_wheel_front;
                BufferedVals.w_wheel_rear = BufferedVals.w_wheel_rear + CurrentVals.w_wheel_rear;
                BufferedVals.v_body = BufferedVals.v_body + CurrentVals.v_body;
                BufferedVals.x_body = BufferedVals.x_body + CurrentVals.x_body;
                BufferedVals.th_wheel_front = BufferedVals.th_wheel_front + CurrentVals.th_wheel_front;
                BufferedVals.th_wheel_rear = BufferedVals.th_wheel_rear + CurrentVals.th_wheel_rear;
                BufferedVals.Fx_front = BufferedVals.Fx_front + CurrentVals.Fx_front;
                BufferedVals.Fz_front = BufferedVals.Fz_front + CurrentVals.Fz_front;
                BufferedVals.Fx_rear = BufferedVals.Fx_rear + CurrentVals.Fx_rear;
                BufferedVals.Fz_rear = BufferedVals.Fz_rear + CurrentVals.Fz_rear;
                BufferedVals.N = BufferedVals.N + 1;
                if (SimTime - LastDrawSimTime > DrawDeltaTime)
                    while (now - LastDrawTime < DrawDeltaTime/(24*3600))
                        pause(.001);
                    end
                    fprintf('%f\t', (now - LastDrawTime)*24*3600);
                    updateView();
                    drawnow;
                    fprintf('%f\n', (now - LastDrawTime)*24*3600);
                    LastDrawTime = LastDrawTime + DrawDeltaTime/(24*3600);
                    LastDrawSimTime = SimTime;
                end
            end
            % keyboard
        else
            error('No automation loaded');
        end
    end

    function simulateFromJoystick
        j = HebiJoystick(1);
        f1 = fopen(OutputFileName, 'a');
        fprintf(f1, 't,clc_p,acc_p,brk_p,gear,w_engine,w_wheel,v_body\n');
        exitCondition = 0;
        LastDrawTime = now;
        SimTime = 0.0;
        LastGearChange = 0.0;
        IntermediateSteps = round(DrawDeltaTime / SimulationDeltaTime); % how many intermediate simulation steps.
        gear_p = 1;
        LastDrawTime2 = now;
        while exitCondition == 0
            b = j.button;
            ax = j.read;
            if b(1) == 1 % first button pressed
                exitCondition = 1;
                break;
            end
            if (SimTime - LastGearChange > 0.5) % TODO: Move this hardcoded thing from here
                if b(7) == 1
                    gear_p = gear_p - 1;
                    LastGearChange = SimTime;
                elseif b(8) == 1
                    gear_p = gear_p + 1;
                    LastGearChange = SimTime;
                end
            end
            clc_p = abs(ax(2)); % use left handle as clutch.
            if ax(3) < .05
                acc_p = abs(ax(3));
                brk_p = 0;
            else
                brk_p = abs(ax(3));
                acc_p = 0;
            end
            Car.drivetrain.controls.set_all(acc_p, brk_p, clc_p, gear_p);
            for i_step = 1:IntermediateSteps % perform a chunk of integrations.
                cvals = [CurrentVals.w_engine; CurrentVals.w_clutch; ...
                    CurrentVals.w_wheel_front; CurrentVals.w_wheel_rear; ...
                    CurrentVals.v_body];
                [av, a_params] = Car.get_acc(cvals);
                avi = integrate_cgs(cvals, av, SimulationDeltaTime);
                % If writing, do it here.
                CurrentVals.w_engine = avi(1);
                CurrentVals.w_clutch = avi(2);
                CurrentVals.w_wheel_front = avi(3);
                CurrentVals.w_wheel_rear = avi(4);
                CurrentVals.v_body = avi(5);
                CurrentVals.x_body = CurrentVals.x_body + SimulationDeltaTime*CurrentVals.v_body;
                CurrentVals.th_wheel_front = CurrentVals.th_wheel_front + SimulationDeltaTime*CurrentVals.w_wheel_front;
                CurrentVals.th_wheel_rear = CurrentVals.th_wheel_rear + SimulationDeltaTime*CurrentVals.w_wheel_rear;
                CurrentVals.Fx_front = a_params.Fx_a;
                CurrentVals.Fz_front = a_params.Fz_a;
                CurrentVals.Fx_rear = a_params.Fx_p;
                CurrentVals.Fz_rear = a_params.Fz_p;
                SimTime = SimTime + SimulationDeltaTime;
                BufferedVals.w_engine = BufferedVals.w_engine + CurrentVals.w_engine;
                BufferedVals.w_clutch = BufferedVals.w_clutch + CurrentVals.w_clutch;
                BufferedVals.w_wheel_front = BufferedVals.w_wheel_front + CurrentVals.w_wheel_front;
                BufferedVals.w_wheel_rear = BufferedVals.w_wheel_rear + CurrentVals.w_wheel_rear;
                BufferedVals.v_body = BufferedVals.v_body + CurrentVals.v_body;
                BufferedVals.x_body = BufferedVals.x_body + CurrentVals.x_body;
                BufferedVals.th_wheel_front = BufferedVals.th_wheel_front + CurrentVals.th_wheel_front;
                BufferedVals.th_wheel_rear = BufferedVals.th_wheel_rear + CurrentVals.th_wheel_rear;
                BufferedVals.Fx_front = BufferedVals.Fx_front + CurrentVals.Fx_front;
                BufferedVals.Fz_front = BufferedVals.Fz_front + CurrentVals.Fz_front;
                BufferedVals.Fx_rear = BufferedVals.Fx_rear + CurrentVals.Fx_rear;
                BufferedVals.Fz_rear = BufferedVals.Fz_rear + CurrentVals.Fz_rear;
                BufferedVals.N = BufferedVals.N + 1;
            end
            fprintf('Computing time: %f\n', (now - LastDrawTime2)*24*3600);
            while (now - LastDrawTime < DrawDeltaTime/(24*3600))
                pause(.001);
            end
            updateView();
            drawnow;
            LastDrawTime2 = now;
            LastDrawTime = LastDrawTime + DrawDeltaTime/(24*3600);
            %LastDrawSimTime = SimTime;
        end
        fclose(f1);
    end

    function updateView()
        ValsToShow.w_engine = BufferedVals.w_engine / BufferedVals.N;
        ValsToShow.w_clutch = BufferedVals.w_engine / BufferedVals.N;
        ValsToShow.w_wheel_rear = BufferedVals.w_wheel_rear / BufferedVals.N;
        ValsToShow.w_wheel_front = BufferedVals.w_wheel_front / BufferedVals.N;
        ValsToShow.v_body = BufferedVals.v_body / BufferedVals.N;
        ValsToShow.x_body = BufferedVals.x_body / BufferedVals.N;
        ValsToShow.th_wheel_rear = BufferedVals.th_wheel_rear / BufferedVals.N;
        ValsToShow.th_wheel_front = BufferedVals.th_wheel_front / BufferedVals.N;
        ValsToShow.Fx_front = BufferedVals.Fx_front / BufferedVals.N;
        ValsToShow.Fz_front = BufferedVals.Fz_front / BufferedVals.N;
        ValsToShow.Fx_rear = BufferedVals.Fx_rear / BufferedVals.N;
        ValsToShow.Fz_rear = BufferedVals.Fz_rear / BufferedVals.N;
        SpeedPanel_Speed_Body_Val.String = num2str(3.6*ValsToShow.v_body);
        SpeedPanel_Speed_Front_Val.String = num2str(3.6*ValsToShow.w_wheel_front*Car.wheel_front.R);
        SpeedPanel_Speed_Rear_Val.String = num2str(3.6*ValsToShow.w_wheel_rear*Car.wheel_rear.R);
        % SpeedPanel_Speed_Engine_Val.String = num2str(rads2rpm(CurrentVals.w_engine));
        SpeedPanel_Engine.set_val(rads2rpm(ValsToShow.w_engine));
        CtrlPanel_Acc.set_val(Car.drivetrain.controls.gas_pedal);
        CtrlPanel_Brk.set_val(Car.drivetrain.controls.brk_pedal);
        CtrlPanel_Clc.set_val(Car.drivetrain.controls.clc_pedal);
        CtrlPanel_Gear.set_val(Car.drivetrain.controls.gear_lever);
        CarView.car_position = [ValsToShow.x_body*1000, 0];
        CarView.wheel_front_th = ValsToShow.th_wheel_front;
        CarView.wheel_rear_th = ValsToShow.th_wheel_rear;
        CarView.Fx_front = ValsToShow.Fx_front;
        CarView.Fz_front = ValsToShow.Fz_front;
        CarView.Fx_rear = ValsToShow.Fx_rear;
        CarView.Fz_rear = ValsToShow.Fz_rear;
        CarView.update_all();
        resetBuffer();
    end


    function resetBuffer()
        fnab = fieldnames(BufferedVals);
        for i_f = 1:length(fnab)
            BufferedVals.(fnab{i_f}) = 0;
        end
    end

end
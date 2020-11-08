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
    'th_wheel_front', 0, 'th_wheel_rear', 0, ...
    'ClutchTorque', 0, 'EngineTorque', 0, 'WheelTorques', [0 0 0 0], ...
    'DiffTorque', 0, 'BrakeTorquesActual', [0 0 0 0], ...
    'BrakeTorquesTheor', [0 0 0 0], 'ClutchEngaged', 0);
BufferedVals = CurrentVals;
BufferedVals.N = 0;
ValsToShow = CurrentVals;
AutomationTrack = load('dummy_auto1.mat');
SimulationDeltaTime = 1/240; %s
DrawDeltaTime = 1/30; %s
SimTime = 0.0;
LastDrawTime = now;
LastDrawSimTime = -1;
OutputFileName = ['logs\debug_', datestr(now, 'yyyy_mm_dd_HH_MM_SS'), '.txt'];
SteeringWheel = false; % a steering wheel is connected.

% Global objects (such as drivetrain, etc)
% TODO: define function to load and save a car's properties
Car = generate_dummy_car_2ax();
Car.dt = SimulationDeltaTime;
av = zeros(5, 1);


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
f1 = fopen(OutputFileName, 'a');
% drawnow;
% simulateFromJoystick();

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
                [av, a_params] = Car.get_acc(cvals, av);
                cvals(2) = a_params.w_clutch; % re-correct for locking
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
                updateBuffer();
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
        if strcmp(j.Name, 'SideWinder Force Feedback Wheel (USB)')
            SteeringWheel = true;
            disp('Recognized steering wheel.');
        end
        fprintf(f1, 'T,t,clc_p,acc_p,brk_p,gear,w_engine,w_clutch,w_wheel_f,w_wheel_r,v_body,Fz_front,Fz_rear,Fx_front,Fx_rear,distance,M_clutch,M_engine,M_wheel_fl,'); 
        fprintf(f1, 'M_wheel_fr,M_wheel_rl,M_wheel_rr,M_diff,M_brake_act_fl,M_brake_act_fr,M_brake_act_rl,M_brake_act_rr,M_brake_th_fl,M_brake_th_fr,M_brake_th_rl,');
        fprintf(f1, 'M_brake_th_rr,clutch_engaged\n');
        %fclose(f1);
        exitCondition = 0;
        LastDrawTime = now;
        SimTime = 0.0;
        LastGearChange = 0.0;
        IntermediateSteps = round(DrawDeltaTime / SimulationDeltaTime); % how many intermediate simulation steps.
        gear_p = 0;
        LastDrawTime2 = now;
        while exitCondition == 0
            b = j.button;
            ax = j.read;
            if ~SteeringWheel
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
            if abs(ax(2)) < .5 % Dead zone.
                clc_p = 0.0;
            else
                clc_p = 2*(abs(ax(2))-.5); % use left handle as clutch.
            end
            if ax(3) < .05
                acc_p = abs(ax(3));
                brk_p = 0;
            else
                brk_p = abs(ax(3));
                acc_p = 0;
            end
            else % Steering Wheel
                if b(1) == 1
                    exitCondition = 1;
                    break;
                end
                if (SimTime - LastGearChange > 0.2) % TODO: Move this hardcoded thing from here
                    if b(7) == 1
                        gear_p = gear_p - 1;
                        LastGearChange = SimTime;
                    elseif b(8) == 1
                        gear_p = gear_p + 1;
                        LastGearChange = SimTime;
                    end
                end
                acc_pedal = -.5*ax(2) + .5;
                brk_pedal = -.5*ax(3) + .5;
                if (acc_pedal < 0.2)
                    acp1 = 0.0;
                else
                    acp1 = (acc_pedal - .2)/.8;
                end
                if (brk_pedal < 0.2)
                    brp1 = 0.0;
                else
                    brp1 = (brk_pedal - .2)/.8;
                end
                if (b(3) == 1) % brake as clutch
                    brk_p = 0.0;
                    clc_p = brp1;
                else
                    if (b(4) == 1)
                        clc_p = 1.0;
                    else
                        clc_p = 0.0;
                    end
                    brk_p = brp1;
                end
                acc_p = acp1;
            end
            Car.drivetrain.controls.set_all(acc_p, brk_p, clc_p, gear_p);
            for i_step = 1:IntermediateSteps % perform a chunk of integrations.
                cvals = [CurrentVals.w_engine; CurrentVals.w_clutch; ...
                    CurrentVals.w_wheel_front; CurrentVals.w_wheel_rear; ...
                    CurrentVals.v_body];
                [av, a_params] = Car.get_acc(cvals, av);
                cvals(2) = a_params.w_clutch; % re-correct for locking
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
                CurrentVals.ClutchTorque = a_params.ClutchTorque;
                CurrentVals.EngineTorque = a_params.EngineTorque;
                CurrentVals.WheelTorques = a_params.WheelTorques;
                CurrentVals.DiffTorque = a_params.DiffTorque;
                CurrentVals.BrakeTorquesActual = a_params.BrakeTorquesActual;
                CurrentVals.BrakeTorquesTheor = a_params.BrakeTorquesTheor;
                CurrentVals.ClutchEngaged = a_params.ClutchEngaged;
                SimTime = SimTime + SimulationDeltaTime;
                updateBuffer();
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
        %fclose(f1);
    end

    function updateView()
        getValsToShow();
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
        %f1 = fopen(OutputFileName, 'a');
        str2print = [datestr(now, 'HH:MM:SS.FFF'), ',' num2str(SimTime), ...
            ',', num2str(Car.drivetrain.controls.clc_pedal), ...
            ',', num2str(Car.drivetrain.controls.gas_pedal), ...
            ',', num2str(Car.drivetrain.controls.brk_pedal), ...
            ',', num2str(Car.drivetrain.controls.gear_lever), ...
            ',', num2str(ValsToShow.w_engine), ...
            ',', num2str(ValsToShow.w_clutch), ...
            ',', num2str(ValsToShow.w_wheel_front), ...
            ',', num2str(ValsToShow.w_wheel_rear), ...
            ',', num2str(ValsToShow.v_body), ...
            ',', num2str(ValsToShow.Fz_front), ...
            ',', num2str(ValsToShow.Fz_rear), ...
            ',', num2str(ValsToShow.Fx_front), ...
            ',', num2str(ValsToShow.Fx_rear), ...
            ',', num2str(ValsToShow.x_body), ...
            ',', num2str(ValsToShow.ClutchTorque), ...
            ',', num2str(ValsToShow.EngineTorque), ...
            ',', num2str(ValsToShow.WheelTorques(1)), ...
            ',', num2str(ValsToShow.WheelTorques(2)), ...
            ',', num2str(ValsToShow.WheelTorques(3)), ...
            ',', num2str(ValsToShow.WheelTorques(4)), ...
            ',', num2str(ValsToShow.DiffTorque), ...
            ',', num2str(ValsToShow.BrakeTorquesActual(1)), ...
            ',', num2str(ValsToShow.BrakeTorquesActual(2)), ...
            ',', num2str(ValsToShow.BrakeTorquesActual(3)), ...
            ',', num2str(ValsToShow.BrakeTorquesActual(4)), ...
            ',', num2str(ValsToShow.BrakeTorquesTheor(1)), ...
            ',', num2str(ValsToShow.BrakeTorquesTheor(2)), ...
            ',', num2str(ValsToShow.BrakeTorquesTheor(3)), ...
            ',', num2str(ValsToShow.BrakeTorquesTheor(4)), ...
            ',', num2str(ValsToShow.ClutchEngaged), ...
            '\n'];
        %fprintf(f1, 'T,t,clc_p,acc_p,brk_p,gear,w_engine,w_clutch,w_wheel_f,w_wheel_r,v_body,Fz_front,Fz_rear,Fx_front,Fx_rear,distance,M_clutch,M_engine,M_wheel_fl'); 
        %fprintf(f1, 'M_wheel_fr,M_wheel_rl,M_wheel_rr,M_diff,M_brake_act_fl,M_brake_act_fr,M_brake_act_rl,M_brake_act_rr,M_brake_th_fl,M_brake_th_fr,M_brake_th_rl,M_brake_th_rr\n');
        fprintf(f1, str2print);
        %fclose(f1);
        resetBuffer();
    end

    function updateBuffer()
        fnab = fieldnames(CurrentVals);
        for i_f = 1:length(fnab)
            BufferedVals.(fnab{i_f}) = BufferedVals.(fnab{i_f}) + CurrentVals.(fnab{i_f});
        end
        BufferedVals.N = BufferedVals.N + 1;
    end

    function getValsToShow()
        fnab = fieldnames(CurrentVals);
        for i_f = 1:length(fnab)
            ValsToShow.(fnab{i_f}) = BufferedVals.(fnab{i_f})./ BufferedVals.N;
        end
    end

    function resetBuffer()
        fnab = fieldnames(BufferedVals);
        for i_f = 1:length(fnab)
            BufferedVals.(fnab{i_f}) = 0;
        end
    end

end
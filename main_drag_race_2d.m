clear all
close all
clc

%% 0-400m simulation
% Patrucco, 2021

%% Define simulation parameters
finish_line = 400.0; % [m]
limit_speed = Inf; % [km/h]
t_limit = 6.0; % [s], time limit for the simulation
dt = 1/240.0; % [s], simulation step
rpm_initial = 2000.0;
redline = 7000.0; % [rpm], when supposed to change gear.
stand_still_time = 5.0; % [s], before start
t_gear_change = 0.2; % [s]

param_file = 'testarossa.txt';
OutputFile = ['DragRace_', datestr(now, 'yyyy_mm_dd_HH_MM_SS'), '.csv'];

car = car2wheels_from_file(param_file);
cnth = car.drivetrain.controls; % handle to controls variable inside car class
cnth.brk_pedal = 1.0;
cnth.clc_pedal = 1.0;
cnth.gas_pedal = 1.0;
cnth.gear_lever = 1;
% Calculate limit speed to determine gear changes
rl_rads = redline * (2*pi/60);
f_ratios = car.drivetrain.gearbox.ratios(2:end) * car.drivetrain.differential.final_drive;
w_radius = car.wheel_rear.R;
vmax_ms = rl_rads./f_ratios.*w_radius;
vmax_kmh = vmax_ms.*3.6;


x0 = zeros(5, 1); % position
x1 = zeros(5, 1); % speeds (engine, clutch, front wheel, rear wheel, vehicle (linear))
x1(1) = rpm_initial*(2*pi/60);
x2 = zeros(5, 1);

keep_sim = true;
t = -stand_still_time;

f = fopen(OutputFile, 'a');
print_status(f, t, [], [], [], cnth, [], 1);
fclose(f);
t_last_gear_change = -Inf;
while keep_sim
    % "Driver" AI
    if (t >= 0.0)
        if (t - t_last_gear_change) > t_gear_change
            cnth.clc_pedal = 0.0;
        else
            cnth.clc_pedal = 1.0;
        end
        cnth.brk_pedal = 0.0;
        cnth.gas_pedal = 1.0;
        if x1(5) >= vmax_ms(cnth.gear_lever)
            cnth.gear_lever = cnth.gear_lever + 1;
            cnth.clc_pedal = 1.0;
            t_last_gear_change = t;
        end
    else
        cnth.brk_pedal = 1.0;
        cnth.clc_pedal = 1.0;
        cnth.gas_pedal = 1.0;
    end
    [x2, dap] = car.get_acc(x1, x2);
    dap.delta = 0.0; dap.av = zeros(1, 4); dap.kv = zeros(1, 4);
    dap.Fy = zeros(1, 4); dap.Ax = x2(5); dap.Ay = 0.0; dap.Az = 0.0;
    if ((x1(5) > limit_speed) || (x0(5) > finish_line) || (t > t_limit))
        keep_sim = false;
    end
    xv0a = x5_to_x16(x0);
    xv1a = x5_to_x16(x1);
    xv2a = x5_to_x16(x2);
    x1 = x1 + x2.*dt;
    x0 = x0 + x1.*dt; % rough integration.
    f = fopen(OutputFile, 'a');
    print_status(f, t, xv0a, xv1a, xv2a, cnth, dap, 0);
    fclose(f);
    t = t + dt;
end





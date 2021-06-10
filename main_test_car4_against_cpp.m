clear all
close all
clc

%% Patrucco, 2021
% Test results when integrating a CarWithFourWheels object.

c = car4wheels_from_file('testarossa.txt');

InputFile = 'out_tr_01.txt';
OutputFile = 'MLout_tr_01.txt';
xv_0 = [0; 0; 0; 0; 0; 0; 0; 0; 0.2; 0; 0; 0; 0.75; 0.75; 0.75; 0.75];
xv_1 = zeros(16, 1);
xv_1(1) = rpm2rads(2000.0);
xv_2 = zeros(16, 1);

MAX_TIME = 1.0;
dt_sim = (1/240.0);

hist = table2struct(readtable(InputFile), 'ToScalar', 1);
dt_log = median(diff(hist.t));
ns_sim = round(dt_log / dt_sim); % Number of steps to average.

c.controls.set_all(hist.gas_pedal(1), hist.brk_pedal(1), hist.clc_pedal(1), ...
    hist.gear_lever(1), hist.steering_wheel(1));

t = 0.0;

f = fopen(OutputFile, 'a');
print_status(f, t, [], [], [], c.controls, [], 1);
fclose(f);
i_log = 1;
while t < MAX_TIME
    xv0h = zeros(16, ns_sim);
    xv1h = zeros(16, ns_sim);
    xv2h = zeros(16, ns_sim);
    for i_s = 1:ns_sim
        [xv_2, dap] = c.get_acc(xv_0, xv_1, xv_2);
        xv_0 = update_drivetrain_kinematics_cgs(xv_0, xv_1, dt_sim, false, false);
        xv_1 = update_drivetrain_kinematics_cgs(xv_1, xv_2, dt_sim, true, false);
        xv0h(:, i_s) = xv_0;
        xv1h(:, i_s) = xv_1;
        xv2h(:, i_s) = xv_2;
    end
    i_log = i_log + 1;
    c.controls.set_all(hist.gas_pedal(i_log), hist.brk_pedal(i_log), hist.clc_pedal(i_log), ...
        hist.gear_lever(i_log), hist.steering_wheel(i_log));
    xv0a = mean(xv0h, 2);
    xv1a = mean(xv1h, 2);
    xv2a = mean(xv2h, 2);
    t = t + dt_log;
    f = fopen(OutputFile, 'a');
    print_status(f, t, xv0a, xv1a, xv2a, c.controls, dap, 0);
    fclose(f);
end

h_out = table2struct(readtable(OutputFile), 'ToScalar', 1);

figure;
plot(hist.t, hist.zc_0, 'r-');
grid on; hold on;
plot(h_out.t, h_out.zc_0, 'b-');
title('ZC0');
xlabel('Time');
legend('C++', 'Matlab');
xlim([0.0, MAX_TIME]);

figure;
plot(hist.t, hist.th_m_1, 'r-');
grid on; hold on;
plot(h_out.t, h_out.th_m_1, 'b-');
title('Engine Speed');
xlabel('Time');
legend('C++', 'Matlab');
xlim([0.0, MAX_TIME]);
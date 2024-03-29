clear all
close all
clc

%% Patrucco, 2021
% Test results when integrating a CarWithFourWheels object.

c = car4wheels_from_file('testarossa.txt');
N_Step = 10;

InputFile = 'out_tr_02.txt';
OutputFile = ['MLout_tr_', datestr(now, 'yyyy_mm_dd_HH_MM_SS'), '.txt'];
hist = table2struct(readtable(InputFile), 'ToScalar', 1);

xv_0 = [hist.th_m_0(1); hist.th_c_0(1); hist.th_fl_0(1); ...
    hist.th_fr_0(1); hist.th_rl_0(1); hist.th_rr_0(1); ...
    hist.xc_0(1); hist.yc_0(1); hist.zc_0(1); ...
    hist.rho_0(1); hist.beta_0(1); hist.sigma_0(1); ...
    hist.l_fl_0(1); hist.l_fr_0(1); hist.l_rl_0(1); hist.l_rr_0(1)];
xv_1 = [hist.th_m_1(1); hist.th_c_1(1); hist.th_fl_1(1); ...
    hist.th_fr_1(1); hist.th_rl_1(1); hist.th_rr_1(1); ...
    hist.xc_1(1); hist.yc_1(1); hist.zc_1(1); ...
    hist.rho_1(1); hist.beta_1(1); hist.sigma_1(1); ...
    hist.l_fl_1(1); hist.l_fr_1(1); hist.l_rl_1(1); hist.l_rr_1(1)];
xv_2 = [hist.th_m_2(1); hist.th_c_2(1); hist.th_fl_2(1); ...
    hist.th_fr_2(1); hist.th_rl_2(1); hist.th_rr_2(1); ...
    hist.xc_2(1); hist.yc_2(1); hist.zc_2(1); ...
    hist.rho_2(1); hist.beta_2(1); hist.sigma_2(1); ...
    hist.l_fl_2(1); hist.l_fr_2(1); hist.l_rl_2(1); hist.l_rr_2(1)];

%xv_0 = [0.0; 0; 0; 0; 0; 0; 0; 0; 0.2; 0; 0; 0; 0.75; 0.75; 0.75; 0.75];
%xv_1 = zeros(16, 1);
%xv_1(1) = rpm2rads(2000.0);
%xv_1(2) = 50.42; %xv_1(1); % / (c.drivetrain.differential.final_drive * c.drivetrain.gearbox.ratios(2));
%xv_2 = zeros(16, 1);

MAX_TIME = 20.0;
dt_sim = (1/240.0);

dt_log = median(diff(hist.t));
ns_sim = round(dt_log / dt_sim); % Number of steps to average.

% c.controls.set_all(0.0, 0.0, 0.0, ...
%    0, 0.0);
c.controls.set_all(hist.gas_pedal(1), hist.brk_pedal(1), hist.clc_pedal(1), ...
    hist.gear_lever(1), hist.steering_wheel(1));
for ii = 1:N_Step
        [xv_2, dap] = c.get_acc(xv_0, xv_1, xv_2);
        %% TODO: Verify SINGLE INT STEP
        xv_0 = update_drivetrain_kinematics_cgs(xv_0, xv_1, dt_sim, false, false);
        xv_1 = update_drivetrain_kinematics_cgs(xv_1, xv_2, dt_sim, true, false);
end



% [xv_2, dap] = c.get_acc(xv_0, xv_1, xv_2);

return
t = 0.0;

f = fopen(OutputFile, 'a');
print_status(f, t, [], [], [], c.controls, [], 1);
fclose(f);
i_log = 1;
while t < MAX_TIME
    xv0h = zeros(16, ns_sim);
    xv1h = zeros(16, ns_sim);
    xv2h = zeros(16, ns_sim);
    dap_all = [];
    for i_s = 1:ns_sim
        [xv_2, dap] = c.get_acc(xv_0, xv_1, xv_2);
        xv_0 = update_drivetrain_kinematics_cgs(xv_0, xv_1, dt_sim, false, false);
        xv_1 = update_drivetrain_kinematics_cgs(xv_1, xv_2, dt_sim, true, false);
        xv0h(:, i_s) = xv_0;
        xv1h(:, i_s) = xv_1;
        xv2h(:, i_s) = xv_2;
        dap_all = [dap_all; dap];
    end
    i_log = i_log + 1;
    c.controls.set_all(hist.gas_pedal(i_log), hist.brk_pedal(i_log), hist.clc_pedal(i_log), ...
        hist.gear_lever(i_log), hist.steering_wheel(i_log));
    xv0a = mean(xv0h, 2);
    xv1a = mean(xv1h, 2);
    xv2a = mean(xv2h, 2);
    dapa = average_struct_set(dap_all);
    t = t + dt_log;
    f = fopen(OutputFile, 'a');
    print_status(f, t, xv0a, xv1a, xv2a, c.controls, dapa, 0);
    fclose(f);
end

h_out = table2struct(readtable(OutputFile), 'ToScalar', 1);

figure;
plot(hist.t, hist.xc_0, 'r-');
grid on; hold on;
plot(h_out.t, h_out.xc_0, 'b-');
title('XC0');
xlabel('Time');
legend('C++', 'Matlab');
xlim([0.0, MAX_TIME]);

figure;
plot(hist.t, hist.yc_0, 'r-');
grid on; hold on;
plot(h_out.t, h_out.yc_0, 'b-');
title('YC0');
xlabel('Time');
legend('C++', 'Matlab');
xlim([0.0, MAX_TIME]);

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
plot(hist.t, hist.th_c_1, 'm-');
plot(h_out.t, h_out.th_m_1, 'b-');
plot(h_out.t, h_out.th_c_1, 'c-');
title('Engine/Clutch Speed');
xlabel('Time');
legend('C++ (E)', 'C++ (C)', 'Matlab (E)', 'Matlab (C)');
xlim([0.0, MAX_TIME]);

figure;
plot(hist.t, hist.th_m_2, 'r-');
grid on; hold on;
plot(hist.t, hist.th_c_2, 'm-');
plot(h_out.t, h_out.th_m_2, 'b-');
plot(h_out.t, h_out.th_c_2, 'c-');
title('Engine/Clutch Acc');
xlabel('Time');
legend('C++ (E)', 'C++ (C)', 'Matlab (E)', 'Matlab (C)');
xlim([0.0, MAX_TIME]);


% figure;
% grid on; hold on;
% title('Clutch Speed');
% xlabel('Time');
% legend('C++', 'Matlab');
% xlim([0.0, MAX_TIME]);

figure;
plot(hist.t, hist.th_fl_1, 'r-');
grid on; hold on;
plot(h_out.t, h_out.th_fl_1, 'b-');
title('Front Left wheel Speed');
xlabel('Time');
legend('C++', 'Matlab');
xlim([0.0, MAX_TIME]);

figure;
plot(hist.t, hist.Mc, 'r-');
grid on; hold on;
plot(h_out.t, h_out.Mc, 'b-');
title('Clutch Torque');
xlabel('Time');
legend('C++', 'Matlab');
xlim([0.0, MAX_TIME]);

figure;
plot(hist.t, hist.Md, 'r-');
grid on; hold on;
plot(h_out.t, h_out.Md, 'b-');
title('Diff Torque (downstream of engine)');
xlabel('Time');
legend('C++', 'Matlab');
xlim([0.0, MAX_TIME]);

figure;
plot(hist.t, hist.Me, 'r-');
grid on; hold on;
plot(h_out.t, h_out.Me, 'b-');
title('Engine Torque');
xlabel('Time');
legend('C++', 'Matlab');
xlim([0.0, MAX_TIME]);

figure;
plot(hist.t, hist.clutch_engaged, 'r-');
grid on; hold on;
plot(h_out.t, h_out.clutch_engaged, 'b-');
title('Clutch Engaged');
xlabel('Time');
legend('C++', 'Matlab');
xlim([0.0, MAX_TIME]);

figure;
plot(hist.t, hist.l_fl_0, 'r-');
grid on; hold on;
plot(hist.t, hist.l_fr_0, 'm-');
plot(h_out.t, h_out.l_fl_0, 'b-');
plot(h_out.t, h_out.l_fr_0, 'c-');
title('Front Suspension Height');
xlabel('Time');
legend('C++ (fl)', 'C++ (fr)', 'Matlab (fl)', 'Matlab (fr)');
xlim([0.0, MAX_TIME]);

figure;
plot(hist.t, hist.Fz_fl, 'r-');
grid on; hold on;
plot(hist.t, hist.Fz_fr, 'm-');
plot(h_out.t, h_out.Fz_fl, 'b-');
plot(h_out.t, h_out.Fz_fr, 'c-');
title('Front Vertical Forces');
xlabel('Time');
legend('C++ (fl)', 'C++ (fr)', 'Matlab (fl)', 'Matlab (fr)');
xlim([0.0, MAX_TIME]);


figure;
plot(hist.t, hist.Fz_rl, 'r-');
grid on; hold on;
plot(hist.t, hist.Fz_rr, 'm-');
plot(h_out.t, h_out.Fz_rl, 'b-');
plot(h_out.t, h_out.Fz_rr, 'c-');
title('Rear Vertical Forces');
xlabel('Time');
legend('C++ (rl)', 'C++ (rr)', 'Matlab (rl)', 'Matlab (rr)');
xlim([0.0, MAX_TIME]);


figure;
plot(hist.t, hist.Fx_rl, 'r-');
grid on; hold on;
plot(hist.t, hist.Fx_rr, 'm-');
plot(h_out.t, h_out.Fx_rl, 'b-');
plot(h_out.t, h_out.Fx_rr, 'c-');
title('Rear Longitudinal Forces');
xlabel('Time');
legend('C++ (rl)', 'C++ (rr)', 'Matlab (rl)', 'Matlab (rr)');
xlim([0.0, MAX_TIME]);
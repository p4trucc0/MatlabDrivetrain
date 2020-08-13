clear all
close all
clc

%% Patrucco, 2020
% Debugging tool for the 1D simulation environment

FileName = 'debug_2020_08_13_10_27_18.txt';

a = generate_dummy_car();

sh = table2struct(readtable(FileName), 'ToScalar', 1);

figure; 
p1 = subplot(211);
plot(sh.t, sh.clc_p, 'b-');
hold on;
plot(sh.t, sh.acc_p, 'r-');
grid on;
legend('clutch', 'gas');
p2 = subplot(212);
plot(sh.t, sh.w_engine./(a.drivetrain.gearbox.get_ratio(1)*a.drivetrain.differential.final_drive), 'r-');
hold on; grid on;
plot(sh.t, sh.w_wheel, 'b-');
plot(sh.t, sh.v_body./a.radius, 'k-');
legend('engine', 'wheel', 'body');
ylabel('rad/s');
xlabel('Time [s]');
linkaxes([p1 p2], 'x');



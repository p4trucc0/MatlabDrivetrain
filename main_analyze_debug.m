clear all
close all
clc

%% Patrucco, 2020
% Debugging tool for the 1D simulation environment

FileName = 'debug_2020_08_13_16_03_22.txt';

a = generate_dummy_car();

ModeFirstGear = false;

sh = table2struct(readtable(FileName), 'ToScalar', 1);

if ModeFirstGear
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
else
figure; 
p1 = subplot(311);
plot(sh.t, sh.clc_p, 'b-');
hold on;
plot(sh.t, sh.acc_p, 'r-');
plot(sh.t, sh.brk_p, 'k-');
grid on;
legend('clutch', 'gas', 'brake');
p2 = subplot(312);
plot(sh.t, rads2rpm(sh.w_engine), 'r-');
ylabel('RPM');
yyaxis right;
plot(sh.t, sh.gear, 'b-');
grid on;
p3 = subplot(313);
plot(sh.t, sh.w_wheel, 'r-');
hold on; grid on;
plot(sh.t, sh.v_body./a.radius, 'b-');
legend('wheel', 'body');
ylabel('rad/s');
xlabel('Time [s]');
linkaxes([p1 p2 p3], 'x');
end


clear all
close all
clc

%% Patrucco, 2020
% Test drivetrain classes and their relations

nlambda_v = [0.0 0.5 1.0];
nrpm_v = [0 1000 2000 3000 4000 5000 6000 7000 7100];
ntorque_map = [0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0;
               0.0  20.0  30.0  40.0  30.0  20.0  10.0   5.0   0.0;
               0.0  40.0  70.0 100.0  75.0  50.0  20.0  10.0   0.0];
nJm = 0.28;
nfv0 = 0.0;
nfv1 = 0.0;

m = Engine(nlambda_v, nrpm_v, ntorque_map, nfv0, nfv1, nJm);
f = Clutch;
c = Gearbox([-15.0 10.0 8.0 6.0 4.0 2.0], [.9 .95 .98 .99 .99 .99], true);
d = Differential(2.0);

cnt = Controls;

drt = SimpleDrivetrain(m, f, c, d, cnt);

% 1.0 test revving
cnt.gas_pedal = 1.0; % completely pushing on the gas
cnt.clc_pedal = 1.0; % clutch pedal completely down (not engaged).
w0_engine = rpm2rads(800);
w0_wheel = 0.0;
J_diff = 0.1;

t0 = 0;
t1 = 10; % final time
dt = .01;

t_v = [t0:dt:t1];
w_engine_v = zeros(size(t_v));

w_engine_curr = w0_engine;
for ii = 1:length(t_v)
    w_engine_v(ii) = w_engine_curr;
    [w1_engine, ~] = drt.get_shaft_acc(w0_wheel, w_engine_curr, 0.0, J_diff);
    w_engine_curr = w_engine_curr + w1_engine * dt;
end

figure; plot(t_v, w_engine_v, 'r-'); grid on
xlabel('Time [s]');
ylabel('Engine Speed');
title('Free engine test (rev)');


% Ok; the engine reaches its maximum regime and stands there as expected.

% 2.0 test behavior with some load
diff_torque = 0; %Nm
cnt.gas_pedal = 1.0;
cnt.clc_pedal = 1.0;
cnt.gear_lever = 1.0; % first gear.

w0_engine = rpm2rads(800);
w0_wheel = 0.0;
J_diff = 0.1;

t0 = 0;
t1 = 10; % final time
dt = .01;

t_v = [t0:dt:t1];
w_engine_v = zeros(size(t_v));
w_wheel_v = zeros(size(t_v));

w_engine_curr = w0_engine;
w_wheel_curr = w0_wheel;
for ii = 1:length(t_v)
    if (t_v(ii) > 3.0)
        cnt.clc_pedal = 0.0; % clutch up after 3 seconds
    end
    w_engine_v(ii) = w_engine_curr;
    w_wheel_v(ii) = w_wheel_curr;
    [w1_engine, w1_wheel] = drt.get_shaft_acc(w_wheel_curr, w_engine_curr, diff_torque, J_diff);
    w_engine_curr = w_engine_curr + w1_engine * dt;
    w_wheel_curr = w_wheel_curr + w1_wheel * dt;
end

figure; plot(t_v, w_engine_v, 'r-'); grid on; hold on;
plot(t_v, w_wheel_v, 'b-');
xlabel('Time [s]');
ylabel('Engine Speed');
title('Engine under load');

% Trends seem reasonable.













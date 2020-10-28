function car_handle = generate_dummy_car_2ax()
% 2020, Patrucco
% Dummy function (as there is still no way to save/load/generate a car from
% scratch) to build a handle to a CWW object.

nlambda_v = [0.0 0.5 1.0];
nrpm_v = [0 500 1000 2000 3000 4000 5000 6000 7000 7100 8000];
ntorque_map = 2.25*[100.0 0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0 -100.0;
               100.0 0.0  20.0  30.0  40.0  30.0  20.0  10.0   5.0   0.0 -100.0;
               100.0 0.0  40.0  70.0 75.0  80.0  100.0  70.0  30.0   0.0 -100.0];
nJm = .25;
nfv0 = 0.005;
nfv1 = 1e-7;

m = Engine(nlambda_v, nrpm_v, ntorque_map, nfv0, nfv1, nJm);
f = Clutch(1000.0, 0.0, 0.005);
c = Gearbox([-15.0 3.8 2.235 1.36 0.971 0.763], [.9 .95 .98 .99 .99 .99], true);
d = Differential2Axle_v2(3.562, [.5 .5], 2, 0.0);
b_f = Brake(0.3, .07, .11, pi/4, 60000);
b_r = Brake(0.3, .07, .11, pi/4, 60000);
cnt = Controls;
bs = BrakeSystem2Axle(b_f, b_r, cnt, struct('x', [0 1], 'y_rf', [1 .5]));


tyre_param = struct();
tyre_param.a0 = 1.4660; tyre_param.a1 = -7.4; tyre_param.a2 = 999;
tyre_param.a3 = 2238; tyre_param.a4 = 8; tyre_param.a5 = 0.0150;
tyre_param.a6 = -0.2350; tyre_param.a7 = -0.3000; tyre_param.a8 = -0.0300;
tyre_param.a9 = -1.0000e-03; tyre_param.a10 = -0.1500; tyre_param.a111 = 0;
tyre_param.a112 = -0.2900; tyre_param.a12 = 17.8000; tyre_param.a13 = -2.4000;
tyre_param.b0 = 1.3600; tyre_param.b1 = -40; tyre_param.b2 = 1038;
tyre_param.b3 = 0.306; tyre_param.b4 = 100; tyre_param.b5 = 0.0800;
tyre_param.b6 = -0.0500; tyre_param.b7 = 0.0500; tyre_param.b8 = -0.0250;
tyre_param.b9 = 0.0; tyre_param.b10 = 0.0; tyre_param.c0 = 2.3150;
tyre_param.c1 = -4; tyre_param.c2 = -3; tyre_param.c3 = -1.6000; 
tyre_param.c4 = -6; tyre_param.c5 = 0; tyre_param.c6 = 0;
tyre_param.c7 = 0.0200; tyre_param.c8 = -0.5800; tyre_param.c9 = 0.1800;
tyre_param.c10 = 0.0430; tyre_param.c11 = 0.0480; tyre_param.c12 = -0.0035;
tyre_param.c13 = -0.1800; tyre_param.c14 = 0.1400; tyre_param.c15 = -1.0290;
tyre_param.c16 = 0.2700; tyre_param.c17 = -1.1000; tyre_param.a11 = 0;
tyre_param.a14 = 0; tyre_param.a15 = 0; tyre_param.a16 = 0;
tyre_param.a17 = 0; tyre_param.a18 = 0; tyre_param.a19 = 0;
tyre_param.a20 = 0; tyre_param.b11 = 0; tyre_param.b12 = 0;
tyre_param.b13 = 0; tyre_param.b14 = 0; tyre_param.b15 = 0;
tyre_param.b16 = 0; tyre_param.b17 = 0; tyre_param.b18 = 0;
tyre_param.b19 = 0; tyre_param.b20 = 0; tyre_param.c18 = 0;
tyre_param.c19 = 0; tyre_param.c20 = 0;
tyre_param.b9 = 0;
tyre_param.b10 = 0;

w_f = Wheel(0.3, 2.0, tyre_param);
w_r = Wheel(0.3, 2.0, tyre_param);
drt = Drivetrain2Axle_v2(m, f, c, d, ...
                bs, cnt, w_f, w_r);

car_handle = CarWithTwoWheels(1200, ...
                w_f, w_r, .55, ...
                2, .29, drt);

car_handle.drivetrain.controls.gear_lever = 1;
car_handle.drivetrain.controls.brk_pedal = 1;

end
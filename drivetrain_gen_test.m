clear
close all
clc

%% Patrucco, 2020
% Testing function for drivetrain_gen, to compare results with those of C++ rendition

case_no = 5;

switch (case_no)
  case 1
th1_v = [30.0; 30.0; 30.0; 30.0];
th1d = 30.0;
Mc = -100.0;
Mfv = [0, 0, 0, 0]';
Fxv = [0, 0, 0, 0]';
Jc = 0.5;
Jrv = 1.2*ones(4, 1);
rc = 0;
rd = 0;
t = 10;
ka = .6;
kp = 0;
Rv = .3*ones(4, 1);
brake_check = false;
case 2
th1_v = [30.0; 30.0; 30.0; 30.0];
th1d = 30.0;
Mc = -150.0;
Mfv = [0, 0, 0, 0]';
Fxv = [100, 70, 120, 40]';
Jc = 0.5;
Jrv = 1.2*ones(4, 1);
rc = 0.0001;
rd = 0;
t = 10;
ka = .6;
kp = 0;
Rv = .3*ones(4, 1);
brake_check = false;
case 3
th1_v = [30.0; 30.0; 30.0; 30.0];
th1d = 30.0;
Mc = -150.0;
Mfv = [40, 45, 20, 29]';
Fxv = [100, 70, 120, 40]';
Jc = 0.5;
Jrv = 1.2*ones(4, 1); Jrv(3) = 1.1; Jrv(4) = 1.1;
rc = 0.0001;
rd = 0.0002;
t = 10;
ka = .6;
kp = 0;
Rv = .3*ones(4, 1); Rv(3) = .31; Rv(4) = .31;
brake_check = false;
case 4
th1_v = [30.0; 30.0; 30.0; 30.0];
th1d = 30.0;
Mc = -150.0;
Mfv = [40, 45, 20, 29]';
Fxv = [100, 70, 120, 40]';
Jc = 0.5;
Jrv = 1.2*ones(4, 1); Jrv(3) = 1.1; Jrv(4) = 1.1;
rc = 0.0001;
rd = 0.0002;
t = 10;
ka = .6;
kp = 0;
Rv = .3*ones(4, 1); Rv(3) = .31; Rv(4) = .31;
brake_check = true;
case 5
th1_v = [0.0; 0.0; 0.0; 0.0];
th1d = 0.0;
Mc = -50.0;
Mfv = [45, 45, 29, 29]';
Fxv = [10, 10, 10, 0]';
Jc = 0.5;
Jrv = 1.2*ones(4, 1); Jrv(3) = 1.1; Jrv(4) = 1.1;
rc = 0.0001;
rd = 0.0002;
t = 10;
ka = .8;
kp = 0;
Rv = .3*ones(4, 1); Rv(3) = .31; Rv(4) = .31;
brake_check = true;
end

for diff_setup = 1:12 %1:12
disp(['Testing case ', num2str(diff_setup)]);
[th2_v, th2d, M_v, Md, Mda, Mdp, add_param] = drivetrain_gen(diff_setup, th1_v, th1d, Mc, Mfv, Fxv, Jc, Jrv, rc, rd, t, ka, kp, Rv, brake_check);
disp(th2_v')
disp(th2d)
disp(Md)
disp(Mda)
disp(Mdp)
disp(M_v')
end
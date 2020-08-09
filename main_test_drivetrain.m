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






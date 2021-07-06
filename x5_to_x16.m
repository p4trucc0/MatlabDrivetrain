function x_out = x5_to_x16(xv_in)
% transform the reduced set of parameters of the "CarWithTwoWheels" model
% into the extended one of the multi-body one (to use common printing
% functions)

x_out = zeros(16, 1);
x_out(1:2) = xv_in(1:2);
x_out(3) = xv_in(3);
x_out(4) = xv_in(3);
x_out(5) = xv_in(4);
x_out(6) = xv_in(4);
x_out(7) = xv_in(5);

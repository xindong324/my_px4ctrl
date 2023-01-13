clear
clc
close
%coeff_x = [ 1.54320988e-03 -2.08333333e-02  6.48148148e-02  0.00000000e+00 1.00000000e+00  5.00000000];
%coeff_y = [ -5.67824059e-01  7.85881923e+00 -2.58944438e+01 -4.90499987e+00 0.00000000  0.15];
%coeff_z = [ -1.15740741e-03  1.73611111e-02 -6.94444444e-02  0.00000000e+00 0.00000000  2.50000000];
acc_y = -9.8 * tan(pi/4);

start_state = [0,0,1.0];
end_state = [5,0.15,2.5];
end2_state = [10,0,1.0];

sv_state = [0, 0, 0];

ev_state = [1, 0, 0];
ev2_state = [0, 0, 0];

sa_state = [0, 0, 0];
ea_state = [0, acc_y, 0];
ea2_state = [0, 0, 0];

duration = 10;

A = [ 0,   0,   0,   0, 0, 1;
     duration.^5, duration.^4, duration.^3, duration.^2, duration, 1;
     0,     0,     0,   0, 1, 0;
     5*duration.^4, 4*duration.^3, 3*duration.^2, 2*duration, 1, 0;
     0,     0,     0,   2, 0, 0;
     20*duration.^3, 12*duration.^2, 6*duration, 2, 0, 0];

bx2 =[end_state(1), end2_state(1), ev_state(1), ev2_state(1), ea_state(1), ea2_state(1)];
by2 =[end_state(2), end2_state(2), ev_state(2), ev2_state(2), ea_state(2), ea2_state(2)];
bz2 =[end_state(3), end2_state(3), ev_state(3), ev2_state(3), ea_state(3), ea2_state(3)];

coeff_x = bx2*A^-1;
coeff_y = by2*A^-1;
coeff_z = bz2*A^-1;

t2 = 0:0.001:duration;
t_len = size(t2,2)

t_vec = [t2.^5; t2.^4; t2.^3; t2.^2; t2; 1* ones(size(t2))];
         %5*t2.^4, 4*t2.^3, 3*t2.^2, 2*t2, 1* ones(size(t2)), zeros(size(t2));
         %20*t2.^3, 12*t2.^2, 6*t2, 2* ones(size(t2)), zeros(size(t2)), zeros(size(t2))];
res_pos_x = coeff_x * t_vec;
res_pos_y = coeff_y * t_vec;
res_pos_z = coeff_z * t_vec;

plot(t2, res_pos_y);
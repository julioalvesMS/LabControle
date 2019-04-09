%% Experimento 03

clear; clc; close all;

[~,~]=mkdir('imagens');

addpath(genpath('dados'))
addpath(genpath('funcoes'))
addpath(genpath('simulink'))

%% Dados experimentais
s = tf('s');

load('system.mat');
load('experimental_data_2.mat');

T = 0.002;


standard_size = round(9/T);
start_position = round(0.5/T)+1;

simulation_time = (standard_size-start_position)*T;

prop_s_r = prop_square(start_position:standard_size,1);
prop_s_u = prop_square(start_position:standard_size,2);
prop_s_y = prop_square(start_position:standard_size,3);
prop_s_t = (0:T:(standard_size-start_position)*T)';

prop_t_r = prop_triangular(start_position:standard_size,1);
prop_t_u = prop_triangular(start_position:standard_size,2);
prop_t_y = prop_triangular(start_position:standard_size,3);
prop_t_t = (0:T:(standard_size-start_position)*T)';

zn_s_r = zn_square(start_position:standard_size,1);
zn_s_u = zn_square(start_position:standard_size,2);
zn_s_y = zn_square(start_position:standard_size,3);
zn_s_t = (0:T:(standard_size-start_position)*T)';

zn_t_r = zn_triangular(start_position:standard_size,1);
zn_t_u = zn_triangular(start_position:standard_size,2);
zn_t_y = zn_triangular(start_position:standard_size,3);
zn_t_t = (0:T:(standard_size-start_position)*T)';

pid_s_r = pid_square(start_position:standard_size,1);
pid_s_u = pid_square(start_position:standard_size,2);
pid_s_y = pid_square(start_position:standard_size,3);
pid_s_t = (0:T:(standard_size-start_position)*T)';

pid_t_r = pid_triangular(start_position:standard_size,1);
pid_t_u = pid_triangular(start_position:standard_size,2);
pid_t_y = pid_triangular(start_position:standard_size,3);
pid_t_t = (0:T:(standard_size-start_position)*T)';

%% Principais graficos

% saída x referência
plot_voltage_time_compare_reference(prop_s_r, prop_s_y, prop_s_t, 'Proporcional - Onda Quadrada')
plot_voltage_time_compare_reference(prop_t_r, prop_t_y, prop_t_t, 'Proporcional - Onda Triangular')

plot_voltage_time_compare_reference(zn_s_r, zn_s_y, zn_s_t, 'Ziegler-Nichols - Onda Quadrada')
plot_voltage_time_compare_reference(zn_t_r, zn_t_y, zn_t_t, 'Ziegler-Nichols - Onda Triangular')

plot_voltage_time_compare_reference(pid_s_r, pid_s_y, pid_s_t, 'PID - Onda Quadrada')
plot_voltage_time_compare_reference(pid_t_r, pid_t_y, pid_t_t, 'PID - Onda Triangular')


% Esforço de controle
plot_voltage_time_input(prop_s_u, prop_s_t, 'Proporcional - Onda Quadrada')
plot_voltage_time_input(prop_t_u, prop_t_t, 'Proporcional - Onda Triangular')

plot_voltage_time_input(zn_s_u, zn_s_t, 'Ziegler-Nichols - Onda Quadrada')
plot_voltage_time_input(zn_t_u, zn_t_t, 'Ziegler-Nichols - Onda Triangular')

plot_voltage_time_input(pid_s_u, pid_s_t, 'PID - Onda Quadrada')
plot_voltage_time_input(pid_t_u, pid_t_t, 'PID - Onda Triangular')

%% Graficos analise

plot_voltage_time_compare_controller(prop_s_y, zn_s_y, pid_s_y, pid_s_t, ' Onda Quadrada - Saída')
plot_voltage_time_compare_controller(prop_s_u, zn_s_u, pid_s_u, pid_s_t, ' Onda Quadrada - Esforço de Controle')

plot_voltage_time_compare_controller(prop_t_y, zn_t_y, pid_t_y, pid_t_t, ' Onda Triangular - Saída')
plot_voltage_time_compare_controller(prop_t_u, zn_t_u, pid_t_u, pid_t_t, ' Onda Triangular - Esforço de Controle')

%% Controlador Proporcional - Sisotool

C1 = 0.35875;

U1 = C1/(1+C1*G);
F1 = U1*G;

prop_s_y_m = lsim(F1, prop_s_r, prop_s_t);
prop_s_u_m = lsim(U1, prop_s_r, prop_s_t);
plot_voltage_time_compare_model(prop_s_y, prop_s_t, prop_s_y_m, prop_s_t, 'Proporcional - Onda Quadrada')

prop_t_y_m = lsim(F1, prop_t_r, prop_t_t);
prop_t_u_m = lsim(U1, prop_t_r, prop_t_t);
plot_voltage_time_compare_model(prop_t_y, prop_t_t, prop_t_y_m, prop_t_t, 'Proporcional - Onda Triangular')

%% Controlador PID - Zieger Nichols

Wosc = 86;
Tosc = 2*pi/Wosc;
Td = Tosc/8;
Ti = Tosc/2;
Kp = 2.1327;
tau = 0.01;
C2 = Kp*(1 + Td*s/(tau*s+1) + 1/(Ti*s));

T = 0.002;
Cd2 = c2d(C2, T, 'tustin');

[num, den] = tfdata(Cd2);
Cd2_num = num{1};
Cd2_den = den{1};

[num, den] = tfdata(G);
G_num = num{1};
G_den = den{1};

reference = zn_s_r;
time = zn_s_t;
sim('discrete_controller.slx', simulation_time)
plot_voltage_time_compare_model(zn_s_y, zn_s_t, y(:,2), y(:,1), 'Ziegler-Nichols - Onda Quadrada')

reference = zn_t_r;
time = zn_t_t;
sim('discrete_controller.slx', simulation_time)
plot_voltage_time_compare_model(zn_t_y, zn_t_t, y(:,2), y(:,1), 'Ziegler-Nichols - Onda Triangular')

%% Controlador PID - Projeto

Td = 0.0107;
Ti = 4.8778;
Kp = 0.8790;
tau = 0.01;
C3 = Kp*(1 + Td*s/(tau*s+1) + 1/(Ti*s));

T = 0.002;
Cd3 = c2d(C3, T, 'tustin');

[num, den] = tfdata(Cd3);
Cd2_num = num{1};
Cd2_den = den{1};

[num, den] = tfdata(G);
G_num = num{1};
G_den = den{1};


reference = pid_s_r;
time = pid_s_t;
sim('discrete_controller.slx', simulation_time)
plot_voltage_time_compare_model(pid_s_y, pid_s_t, y(:,2), y(:,1), 'PID - Onda Quadrada')

reference = pid_t_r;
time = pid_t_t;
sim('discrete_controller.slx', simulation_time)
plot_voltage_time_compare_model(pid_t_y, pid_t_t, y(:,2), y(:,1), 'PID - Onda Triangular')

%% Experimento 04

clear; clc; close all;

[~,~]=mkdir('imagens');

addpath(genpath('dados'))
addpath(genpath('funcoes'))
addpath(genpath('simulink'))

%% Dados importantes
s = tf('s');

load('system.mat');
load('exp4.mat');

[num, den] = tfdata(G);
G_num = num{1};
G_den = den{1};

C = 1e-6; % [F]

f = 0.5;
ta = 0:1e-6:10;
ra = 0.5*square(2*pi*ta*f) + 0.5;

%% Leitura dos dados experimento

T = 0.002;

standard_size = round(30/T);
start_position = round(20/T);

simulation_time = (standard_size-start_position)*T;

exp_r = exp4(start_position:standard_size,1);
exp_u = exp4(start_position:standard_size,2);
exp_y = exp4(start_position:standard_size,3)+0.06;
exp_t = (0:T:(standard_size-start_position)*T)';

%% Controlador PID

Td = 0.0107;
Ti = 4.8778;
Kp = 0.8790;
tau = 0.01;
Co = Kp*(1 + Td*s/(tau*s+1) + 1/(Ti*s));
Co = minreal(Co);
sys = pid(Co);

Fo = Co*G/(1+Co*G);

T = 0.002;
Cd = c2d(Co, T, 'tustin');

[num, den] = tfdata(Cd);
Cd2_num = num{1};
Cd2_den = den{1};

SquareWave=1;
sim('ziegler_nichols_2.slx', 10)

%% Resistores

R1_R2 = 1/(sys.ki * C);
R1_R3 = (sys.kp + sys.ki*tau)*R1_R2;
R1R3 = (sys.kd + tau*sys.kp)*R1_R2/C;
R1R2 = tau*R1_R2/C;

%% Sistema ideal

eq = [1 -R1_R2 R1R2];
R1 = abs(min(roots(eq)));
R2 = R1R2/R1;
R3 = R1_R3 - R1;

Ci = (R1*R3*C^2*s^2+ (R1+R3)*C*s+ 1)/(R1*R2*C^2*s^2+ (R1+R2)*C*s);
Ci = minreal(Ci);

Fi = Ci*G/(1+Ci*G);

yi = lsim(Fi, ra, ta);
ui = lsim(Ci/(1+Ci*G), ra, ta);


sistema = 'Controlador Analógico';

name = strcat(sistema, ' - Resposta - Comparação com o PID');
figure;
hold all;
plot(ta, yi);
plot(y(:,1), y(:,2));
ylabel('Tensão [V]');
xlabel('Tempo [s]');
title(name);
legend('Controlador Analógico', 'PID');
hold off;
saveas(gcf, strcat('imagens/', name, '.png'));

plot_voltage_time_input(ui, ta, sistema)

%% Criavel

% Valores Originais
% R1 = 1.0018e+04;
% R2 = 5.5392e+06;
% R3 = 4.8778e+06;

R1 = 1e+04;
R2 = 4.7e+06;
R3 = 4.7e+06;

Cr = (R1*R3*C^2*s^2+ (R1+R3)*C*s+ 1)/(R1*R2*C^2*s^2+ (R1+R2)*C*s);
Cr = minreal(Cr);

Fr = Cr*G/(1+Cr*G);

yr = lsim(Fr, ra, ta);
ur = lsim(Cr/(1+Cr*G), ra, ta);

sistema = 'Controlador Comercial';

name = strcat(sistema, ' - Resposta - Comparação com o teórico');
figure;
hold all;
plot(ta, yr);
plot(ta, yi);
ylabel('Tensão [V]');
xlabel('Tempo [s]');
title(name);
legend('Controlador Comercial', 'Controlador Teórico');
hold off;
saveas(gcf, strcat('imagens/', name, '.png'));

plot_voltage_time_input(ur, ta, sistema)

%%

plot_voltage_time_compare_reference(exp_r, exp_y, exp_t, 'Controlador Experimental')
plot_voltage_time_input(exp_u, exp_t, 'Controlador Experimental')

experimental.r = exp_r;
experimental.u = exp_u;
experimental.y = exp_y;
experimental.t = exp_t;
experimental.name = 'Experimental';

teorico.r = yr;
teorico.u = ur;
teorico.y = yr;
teorico.t = ta;
teorico.name = 'Comercial';

original.r = exp_r;
original.u = exp_u;
original.y = y(:,2);
original.t = y(:,1);
original.name = 'PID';

data{1} = experimental;
data{2} = teorico;
data{3} = original;

plot_voltage_time_compare_controller(data, ' Prático x Teórico')

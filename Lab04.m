%% Experimento 04

clear; clc; close all;

[~,~]=mkdir('imagens');

addpath(genpath('dados'))
addpath(genpath('funcoes'))
addpath(genpath('simulink'))

%% Dados importantes
s = tf('s');

load('system.mat');

[num, den] = tfdata(G);
G_num = num{1};
G_den = den{1};

C = 1e-6; % [F]

f = 0.5;
ta = 0:1e-6:10;
ra = 0.5*square(2*pi*ta*f) + 0.5;

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


sistema = 'Controlador Equivalente';

name = strcat(sistema, ' - Resposta - Comparação com o Objetivo');
figure;
hold all;
plot(ta, yi);
plot(y(:,1), y(:,2));
ylabel('Tensão [V]');
xlabel('Tempo [s]');
title(name);
legend('Controlador Analógico', 'Controlador Objetivo');
hold off;
saveas(gcf, strcat('imagens/', name, '.png'));

plot_voltage_time_input(ui, ta, sistema)

%% Criavel

% Valores Originais
% R1 = 1.0018e+04;
% R2 = 5.5392e+06;
% R3 = 4.8778e+06;

R1 = 1e+04;
R2 = 8.2e+05;
R3 = 8.2e+05;

Cr = (R1*R3*C^2*s^2+ (R1+R3)*C*s+ 1)/(R1*R2*C^2*s^2+ (R1+R2)*C*s);
Cr = minreal(Cr);

Fr = Cr*G/(1+Cr*G);

yr = lsim(Fr, ra, ta);
ur = lsim(Cr/(1+Cr*G), ra, ta);

sistema = 'Controlador Aplicável';

name = strcat(sistema, ' - Resposta - Comparação com o Equivalente');
figure;
hold all;
plot(ta, yr);
plot(ta, yi);
ylabel('Tensão [V]');
xlabel('Tempo [s]');
title(name);
legend('Controlador Aplicável', 'Controlador Equivalente');
hold off;
saveas(gcf, strcat('imagens/', name, '.png'));

plot_voltage_time_input(ur, ta, sistema)
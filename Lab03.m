%% Experimento 03

clear; clc; close all;

[~,~]=mkdir('imagens');

addpath(genpath('dados'))
addpath(genpath('funcoes'))
addpath(genpath('simulink'))

%% Planta a ser controlada
s = tf('s');

load('system.mat');

%% Controlador Proporcional - Sisotool

C1 = 0.35875;
%sisotool(G,C1);

% Sobreelevação - 0%
% Estabilização - 0.248
% Amortecimento - 1
% Erro regime degrau - 0 [V]
% Erro regime rampa - 0.09 [V]

F = C1*G/(1+C1*G);

figure
% Degrau
subplot(2,1,1)
step(F);
title('Controlador Proporcional - Degrau Unitário')
ylabel('Tensão de Saída [V]')
xlabel('Tempo [s]')
% Rampa
subplot(2,1,2)
step(F/s);
title('Controlador Proporcional - Rampa Unitária')
ylabel('Tensão de Saída [V]')
xlabel('Tempo [s]')

name = 'Controlador Proporcional - Resposta Rampa e Degrau';
saveas(gcf, strcat('imagens/', name, '.png'));

%% Analisando saida - Proporcional

f = 0.5;
t = 0:1e-6:5;
r = 0.5*square(2*pi*t*f) + 0.5;
y = lsim(F, r, t);

plot_voltage_time(y,t,'Controlador Proporcional - Onda Quadrada')

r = square(2*pi*t*f);
y = lsim(F/s, r, t);

plot_voltage_time(y,t,'Controlador Proporcional - Onda Triangular')


%% Controlador PID - Zieger Nichols

Kosc = 6.0689;

%sisotool(G, Kosc)

Wosc = 86;
Tosc = 2*pi/Wosc;
Td = Tosc/8;
Ti = Tosc/2;
Kp = 2.1327;
tau = 0.01;
C2 = Kp*(1 + Td*s/(tau*s+1) + 1/(Ti*s));

%sisotool(G, C2)
% Sobreelevação - 79.4%
% Estabilização - 0.309
% Amortecimento - 0.231
% Erro regime degrau - 0 [V]
% Erro regime rampa - 0 [V]

F = C2*G/(1+C2*G);

figure
% Degrau
subplot(2,1,1)
step(F);
title('Ziegler-Nichols - Degrau Unitário')
ylabel('Tensão de Saída [V]')
xlabel('Tempo [s]')
% Rampa
subplot(2,1,2)
step(F/s);
title('Ziegler-Nichols - Rampa Unitária')
ylabel('Tensão de Saída [V]')
xlabel('Tempo [s]')

name = 'Ziegler-Nichols - Resposta Rampa e Degrau';
saveas(gcf, strcat('imagens/', name, '.png'));

%% Analisando saida - Ziegler-Nichols - Discreto

T = 0.002;
Cd2 = c2d(C2, T, 'tustin');

[num, den] = tfdata(Cd2);
Cd2_num = num{1};
Cd2_den = den{1};

[num, den] = tfdata(G);
G_num = num{1};
G_den = den{1};

SquareWave = 1;
sim('ziegler_nichols_2.slx')
plot_voltage_time(y(:,2),y(:,1),'Ziegler-Nichols Discreto - Onda Quadrada')

SquareWave = 0;
sim('ziegler_nichols_2.slx')
plot_voltage_time(y(:,2),y(:,1),'Ziegler-Nichols - Onda Triangular')


%% Controlador PID - Projeto

Kosc = 6.0689;

%sisotool(G, Kosc)

Wosc = 86;
Tosc = 2*pi/Wosc;
Td = 0.0107;
Ti = 4.8778;
Kp = 0.8790;
tau = 0.01;
C3 = Kp*(1 + Td*s/(tau*s+1) + 1/(Ti*s));

%sisotool(G, C3)
% Sobreelevação - 2.24%
% Estabilização - 0.113
% Amortecimento - 0.722
% Erro regime degrau - 0 [V]
% Erro regime rampa - 0 [V]

F = C3*G/(1+C3*G);

figure
% Degrau
subplot(2,1,1)
step(F);
title('PID - Degrau Unitário')
ylabel('Tensão de Saída [V]')
xlabel('Tempo [s]')
% Rampa
subplot(2,1,2)
step(F/s);
title('PID - Rampa Unitária')
ylabel('Tensão de Saída [V]')
xlabel('Tempo [s]')

name = 'PID - Resposta Rampa e Degrau';
saveas(gcf, strcat('imagens/', name, '.png'));

%% Analisando saida - Projeto PID - Discreto

T = 0.002;
Cd3 = c2d(C3, T, 'tustin');

[num, den] = tfdata(Cd3);
Cd2_num = num{1};
Cd2_den = den{1};

[num, den] = tfdata(G);
G_num = num{1};
G_den = den{1};

SquareWave = 1;
sim('ziegler_nichols_2.slx')
plot_voltage_time(y(:,2),y(:,1),'PID Discreto - Onda Quadrada')

SquareWave = 0;
sim('ziegler_nichols_2.slx')
plot_voltage_time(y(:,2),y(:,1),'PID - Onda Triangular')
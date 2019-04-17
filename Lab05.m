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

%% Controlador Proporcional - Sisotool

Cp = 0.35875;
%sisotool(G,Cp);

% Sobreelevação - 0%
% Estabilização - 0.248
% Amortecimento - 1
% Erro regime degrau - 0 [V]
% Erro regime rampa - 0.09 [V]

Fp = Cp*G/(1+Cp*G);

%% Determinar ganho

sG = (2.298e05) / (s^2 + 188.7*s + 7388 );
Kv = evalfr(sG, 0);

K = 3;
evalfr(K*sG, 0);

Fk = K*G/(1+K*G);
%sisotool(G,K);

figure
% Degrau
subplot(2,1,1)
step(Fk, 1);
title('K - Degrau Unitário')
ylabel('Tensão de Saída [V]')
xlabel('Tempo [s]')
% Rampa
subplot(2,1,2)
step(Fk/s, 1);
title('K - Rampa Unitária')
ylabel('Tensão de Saída [V]')
xlabel('Tempo [s]')

name = 'K - Resposta Rampa e Degrau';
saveas(gcf, strcat('imagens/', name, '.png'));

%% Atraso Avanço

%K = 1;
Md = 42;

[~,Mf,~,~] = margin(K*G);

%figure
%margin(K*G)
%saveas(gcf, strcat('imagens/Magem de Ganho.png'));

phi = pi*(Md-Mf)/180;
alpha_v = (1+sin(phi))/(1-sin(phi));

Gain = sqrt(alpha_v);

wg = getGainCrossover(K*G,Gain);

tau_v = 1/(wg*sqrt(alpha_v));

alpha_t = 1/alpha_v;
tau_t = 10*alpha_v*tau_v/alpha_t;

Cv = (alpha_v*tau_v*s+1)/(tau_v*s+1);
Ct = (alpha_t*tau_t*s+1)/(tau_t*s+1);

Cvt = K*Cv*Ct;

%sisotool(G, Cvt)

Fvt = Cvt*G/(1+Cvt*G);
figure
% Degrau
subplot(2,1,1)
step(Fvt, 1);
title('Avanço-Atraso - Degrau Unitário')
ylabel('Tensão de Saída [V]')
xlabel('Tempo [s]')
% Rampa
subplot(2,1,2)
step(Fvt/s, 1);
title('Avanço-Atraso - Rampa Unitária')
ylabel('Tensão de Saída [V]')
xlabel('Tempo [s]')

name = 'Avanço-Atraso - Resposta Rampa e Degrau';
saveas(gcf, strcat('imagens/', name, '.png'));

%%
figure
margin(Fvt);

name = 'Margem - Controlador Avanco-Atraso';
saveas(gcf, strcat('imagens/', name, '.png'));


%% Comparação Degrau

time = 1;

[yp, tp] = step(Fp, time);
prop.y = yp;
prop.t = tp;
prop.name = 'Proporcional';

[ys, ts] = step(Fo, time);
pid.y = ys;
pid.t = ts;
pid.name = 'Sisotool';

[yvt, tvt] = step(Fvt, time);
atav.y = yvt;
atav.t = tvt;
atav.name = 'Atraso-Avanço';

data{1} = prop;
data{2} = pid;
data{3} = atav;

plot_voltage_time_compare_controller(data, ' Degrau - Avanço-Atraso')


%% Comparação Rampa

time = 1;

[yp, tp] = step(Fp/s, time);
prop.y = yp;
prop.t = tp;
prop.name = 'Proporcional';

[ys, ts] = step(Fo/s, time);
pid.y = ys;
pid.t = ts;
pid.name = 'Sisotool';

[yvt, tvt] = step(Fvt/s, time);
atav.y = yvt;
atav.t = tvt;
atav.name = 'Atraso-Avanço';

data{1} = prop;
data{2} = pid;
data{3} = atav;

plot_voltage_time_compare_controller(data, ' Rampa - Avanço-Atraso')

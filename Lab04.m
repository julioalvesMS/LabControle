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

%% Controlador PID

Wosc = 86;
Tosc = 2*pi/Wosc;
Td = Tosc/8;
Ti = Tosc/2;
Kp = 2.1327;
tau = 0.01;
Co = Kp*(1 + Td*s/(tau*s+1) + 1/(Ti*s));
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
R1_R3 = sys.kp*R1_R2;
R1R3 = sys.kd*R1_R2/C;
R1R2 = tau*R1_R2/C;

R1 = 15e3;
R2 = 2.2e3;
R3 = 22e3;

Cr = (R1*R3*C^2*s^2+ (R1+R3)*C*s+ 1)/(R1*R2*C^2*s^2+ (R1+R2)*C*s);
Cr = minreal(Cr);

Fr = Cr*G/(1+Cr*G);

f = 0.5;
tr = 0:1e-6:10;
rr = 0.5*square(2*pi*tr*f) + 0.5;
yr = lsim(Fr, rr, tr);

plot_voltage_time_compare_model(yr, tr, y(:,2), y(:,1), 'Controlador Analógico')
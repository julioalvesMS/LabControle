clear; clc; close all;

[~,~]=mkdir('imagens');

addpath(genpath('dados'))

%% Propriedades da planta

R1 = 100e3; % [Ohm]
R2 = 10e3;  % [Ohm]
R3 = 100e3; % [Ohm]
R4 = 220e3; % [Ohm]
R5 = 100e3; % [Ohm]
R6 = 470e3; % [Ohm]
R7 = 1e6;   % [Ohm]

C1 = 0.1e-6; % [F]
C2 = 0.1e-6; % [F]
C3 = 0.1e-6; % [F]

%% Função de transferência
s = tf('s');

G1 = -R2/R1;
G2 = -R4/(R3 + R3*R4*C1*s);
G3 = -R6/(R5 + R5*R6*C2*s);
G4 = -1/(s*R7*C3);

G2 = minreal(G2);
G3 = minreal(G3);
G4 = minreal(G4);

G = G1 * G2 * G3 * G4;
G = minreal(G);

%% Análises

margin(G);
saveas(gcf, strcat('imagens/', 'Bode de Ganho e de Fase', '.png'));

t=0:1e-3:4; %[s]
f=1;    % [Hz]
u=(square(2*pi*f*t)+1)/2;   % [V]

y1 = G1*u;
y2 = lsim(G1*G2, u, t);
y3 = lsim(G1*G2*G3, u, t);
y4 = lsim(G, u, t);

plot_voltage_time(y1, t, 'Amplificador 1');
plot_voltage_time(y2, t, 'Amplificador 2');
plot_voltage_time(y3, t, 'Amplificador 3');
plot_voltage_time(y4, t, 'Amplificador 4');

%% Experimento

load('y.mat');

t1 = 0:1e-3:length(y1)/1e3 - 1e-3;
t2 = 0:1e-3:length(y2)/1e3 - 1e-3;
t3 = 0:1e-3:length(y3)/1e3 - 1e-3;
t4 = 0:1e-3:length(y4)/1e3 - 1e-3;

u1 = y1(:,2);
u2 = y2(:,2);
u3 = y3(:,2);
u4 = y4(:,2);

y1 = y1(:,1);
y2 = y2(:,1);
y3 = y3(:,1);
y4 = y4(:,1);

plot_voltage_time(y1, t1, 'Amplificador Real 1');
plot_voltage_time(y2, t2, 'Amplificador Real 2');
plot_voltage_time(y3, t3, 'Amplificador Real 3');
plot_voltage_time(y4, t4, 'Amplificador Real 4');


%% Identificação da planta


% =========== G1 ===========
idx_to_keep = find(u1 > 0);
k1 = mean(y1(idx_to_keep)./u1(idx_to_keep));
G1_real = k1;
y1_model = G1_real*u1;

plot_voltage_time_compare([y1 y1_model], t1, 'Amplificador 1');


% =========== G2 ===========
u2_G2 = u2 * G1_real;

idx_to_keep = find(y2 > max(y2)*0.98);
y2_reg = y2(idx_to_keep);
u2_reg = u2_G2(idx_to_keep);
idx_to_keep = find(u2_reg < 0);

k2 = mean(y2_reg(idx_to_keep)./u2_reg(idx_to_keep));
tal2 = 0.0075;

G2_real = minreal(k2/(tal2*s+1));
y2_model = lsim(G1_real*G2_real, u2, t2);

plot_voltage_time_compare([y2 y2_model], t2, 'Amplificador 2');


% =========== G3 ===========
u3_G3 = lsim(G1_real * G2_real, u3, t3);

idx_to_keep = find(y3 < min(y3)*0.98);
y3_reg = y3(idx_to_keep);
u3_reg = u3_G3(idx_to_keep);
idx_to_keep = find(u3_reg > 0);

k3 = mean(y3_reg(idx_to_keep)./u3_reg(idx_to_keep));

k = k2*k3;

yt1 = -0.14;
yt2 = -3.343;

a = yt1/k;
b = yt2/k;

p(1) = 1;
p(2) = -(1-b-exp(-2))/(1-a-exp(-1));
p(3) = ((1-b)*exp(-1) - (1-a)*exp(-2))/(1-a-exp(-1));

x = roots(p);
x_tal = x(1);

tal3 = -tal2/log(x_tal);

G3_real = minreal(k3/(-tal3*s+1));
y3_model = lsim(G1_real*G2_real*G3_real, u3, t3);

plot_voltage_time_compare([y3 y3_model], t3, 'Amplificador 3');


% =========== G4 ===========
u4_G4 = lsim(G1_real * G2_real * G3_real, u4, t4);

p1 = 2.625;
p2 = -6.083;
dt = 0.861-0.581;

u_reg = 0.9979;

k4 = (p2-p1)/(dt*u_reg);

G4_real = k4/s;
y4_model = lsim(G1_real * G2_real * G3_real*G4_real, u4, t4);

plot_voltage_time_compare([y4 y4_model], t4, 'Amplificador 4');


% =========== G ===========
G_real = G1_real * G2_real * G3_real*G4_real;
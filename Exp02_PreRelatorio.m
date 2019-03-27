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

plot_voltage_time(y1, t1, 'Amplificador 1');
plot_voltage_time(y2, t2, 'Amplificador 2');
plot_voltage_time(y3, t3, 'Amplificador 3');
plot_voltage_time(y4, t4, 'Amplificador 4');

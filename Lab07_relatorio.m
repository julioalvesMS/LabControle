%% Experimento 06

clear; clc; close all;

[~,~]=mkdir('imagens');

addpath(genpath('dados'))
addpath(genpath('funcoes'))
addpath(genpath('simulink'))

%% Dados do Sistema
s = tf('s');

load('system.mat');

[num, den] = tfdata(G);
G_num = num{1};
G_den = den{1};


%% Experimento

load('exp7_rho1.csv')
load('exp7_rho2.csv')


T = 0.002;

% Proporcional
standard_size = round(12/T);
start_position = round(2/T);

exp_rho1.r = exp7_rho1(start_position:standard_size,1);
exp_rho1.u = exp7_rho1(start_position:standard_size,2);
exp_rho1.y = exp7_rho1(start_position:standard_size,3);
exp_rho1.t = (0:T:(standard_size-start_position)*T)';
exp_rho1.name = '\rho = 1';

exp_rho2.r = exp7_rho2(start_position:standard_size,1);
exp_rho2.u = exp7_rho2(start_position:standard_size,2);
exp_rho2.y = exp7_rho2(start_position:standard_size,3);
exp_rho2.t = (0:T:(standard_size-start_position)*T)';
exp_rho2.name = 'Lugar das Raízes';



%% Sistema em SS

[A, B, C, D] = tf2ss(G_num, G_den);
sys_o = ss(A, B, C, D);

sys_c = canon(sys_o, 'companion');

sys = ss(sys_c.A', flipud(sys_c.B), fliplr(sys_c.C), sys_c.D);

A = sys.A;
B = sys.B;
C = sys.C;
D = sys.D;

%% Primeira Parte - Cálculos

Q = C'*C;

rho=1;
[K, P, E] = lqr(sys, Q, rho);

pole = min(real(E));

po = ones(size(E))*pole*4;
L = acker(A', C', po)';

S = C/(s*eye(length(E)) - (A-B*K))*B;
S0 = evalfr(S,0);

m3 = 0;
m2 = 0;
m1 = 1/(S0*K(1));
M = [m1 m2 m3 ]';

F1 = S*K*M;

%% Segunda Parte - Cálculos

V = C;

phi = V/(s*eye(length(A)) - A)*B;

Gs = phi'*phi;

%sisotool(phi'*phi,rho^-1);

rho = 1/0.082617;

F = 1+rho^-1*Gs;
poles = zero(F);
p = poles(poles<0);

K_cal = acker(A, B, p);
[K_lqr, ~, E] = lqr(sys, V'*V, rho);

pole = min(real(E));

po = ones(size(E))*pole*4;
L = acker(A', C', po)';

S = C/(s*eye(length(E)) - (A-B*K_lqr))*B;
S0 = evalfr(S,0);

m3 = 0;
m2 = 0;
m1 = 1/(S0*K_lqr(1));
M = [m1 m2 m3 ]';

F2 = S*K_cal*M;

%% Comparação com o teórico

teo_rho1.y = lsim(F1, exp_rho1.r, exp_rho1.t);
teo_rho1.t = exp_rho1.t;

teo_rho2.y = lsim(F2, exp_rho2.r, exp_rho2.t);
teo_rho2.t = exp_rho2.t;


%% Gráficos

plot_voltage_time_input(exp_rho1.u, exp_rho1.t, 'Projeto com \rho = 1')
plot_voltage_time_input(exp_rho2.u, exp_rho2.t, 'Projeto por lugar das raízes')

plot_voltage_time_compare_reference(exp_rho1.r, exp_rho1.y, exp_rho1.t, 'Projeto com \rho = 1')
plot_voltage_time_compare_reference(exp_rho2.r, exp_rho2.y, exp_rho2.t, 'Projeto por lugar das raízes')

plot_voltage_time_compare_model(exp_rho1.y, exp_rho1.t, teo_rho1.y, teo_rho1.t, 'Projeto com \rho = 1')
plot_voltage_time_compare_model(exp_rho2.y, exp_rho2.t, teo_rho2.y, teo_rho2.t, 'Projeto por lugar das raízes')

data{1} = exp_rho1;
data{2} = exp_rho2;
plot_voltage_time_compare_controller(data, 'Resposta à Onda Quadrada')
plot_voltage_time_compare_controller_input(data, 'Esforço de Controle')

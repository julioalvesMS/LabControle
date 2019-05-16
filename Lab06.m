%% Experimento 06

clear; clc; close all;

[~,~]=mkdir('imagens');

addpath(genpath('dados'))
addpath(genpath('funcoes'))
addpath(genpath('simulink'))

%% Dados importantes
s = tf('s');

load('system.mat');
G_old = G;

% Planta varlos
G_v = 25070/(s^3 + 97.6*s^2 + 1986*s);

G_f = 9.681e05/(s^3 + 1057*s^2 + 5.676e04*s);

G_n = 24838/(s^3 + 100.7*s^2 + 1971*s);
G = G_n;
G=G_old;

[num, den] = tfdata(G);
G_num = num{1};
G_den = den{1};

f = 0.5;
ta = 0:1e-6:10;
ra = 0.5*square(2*pi*ta*f) + 0.5;

%%

[A, B, C, D] = tf2ss(G_num, G_den);
sys_o = ss(A, B, C, D);

sys_c = canon(sys_o, 'companion');

sys = ss(sys_c.A', flipud(sys_c.B), fliplr(sys_c.C), sys_c.D);

A = sys.A;
B = sys.B;
C = sys.C;
D = sys.D;

%% Projeto controlador

te = 0.5;
qsi = sqrt(2)/2;

wn = log(0.02)/(qsi*te);

p = [-30 -roots([1 2*qsi*wn wn^2])'];

K = acker(sys.A, sys.B, p);

S = sys.C/(s*eye(length(p)) - (sys.A-sys.B*K))*sys.B;
S0 = evalfr(S,0);

%% Erro nulo entrada degrau
syms s_sym m1 m2 m3
M_sym = [m1 m2 m3 ]';
eq1 = S0*K*M_sym == 1;
res = solve([eq1]);

m3 = 0;
m2 = 0;
m1 = eval(res);
M1 = [m1 m2 m3 ]';

%% Erro nulo entrada rampa

[num,den] = tfdata(S);
syms s_sym m1 m2 tau
S_sym = simplify(poly2sym(cell2mat(num),s_sym)/poly2sym(cell2mat(den),s_sym));
M0 = (s_sym/tau + 1)/(s_sym/30 + 1);

m3 = 0;

Sd = diff(S_sym*M0, s_sym);
s_sym = 0;
Sd0 = eval(Sd);

eq2 = Sd0*K*M1 == 0;

res = solve(eq2);

tau = eval(res);

M2 = [m1 m2  m3 ]';
%% Projeto Observador

po = [-100 -100 -100];
L = acker(sys.A', sys.C', po)';

%%

Cu = tf(ss(sys.A-L*sys.C, sys.B, K, 0));
Cy = tf(ss(sys.A-L*sys.C, L, K, 0));

%% Simulação

M0 = (s/tau + 1)/(s/30 + 1);

M = M1;
Ms = M0;
[Ms_num, Ms_den] = tfdata(Ms);

h = K*M;
hs = h*Ms;

%%


x0 = [0 0 0]';


SquareWave = 1;
sim('state_feedback_controller.slx', 4)

degrau.t = y(:,1);
degrau.y = y(:,2);
degrau.r = r(:,2);
degrau.u = u(:,2);


SquareWave = 0;
sim('state_feedback_controller.slx', 4)

rampa.t = y(:,1);
rampa.y = y(:,2);
rampa.r = r(:,2);
rampa.u = u(:,2);

%% Gráficos

% Degrau
plot_voltage_time_compare_reference(degrau.r, degrau.y, degrau.t, 'Entrada Degrau')
plot_voltage_time_input(degrau.u, degrau.t, 'Entrada Degrau')

% Rampa
plot_voltage_time_compare_reference(rampa.r, rampa.y, rampa.t, 'Entrada Rampa')
plot_voltage_time_input(rampa.u, rampa.t, 'Entrada Rampa')


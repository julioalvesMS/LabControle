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


%% Leitura dos dados experimento 6 - Realimentação de Estados

load('Go_M.csv')
load('Go_Ms.csv')
load('Go_degrau.csv')

T = 0.002;

% Proporcional
standard_size = round(12/T);
start_position = round(2/T);

exp_M_qua.r = Go_M(start_position:standard_size,1);
exp_M_qua.u = Go_M(start_position:standard_size,2);
exp_M_qua.y = Go_M(start_position:standard_size,3);
exp_M_qua.t = (0:T:(standard_size-start_position)*T)';
exp_M_qua.name = 'Realimentação M';

exp_Ms_qua.r = Go_Ms(start_position:standard_size,1);
exp_Ms_qua.u = Go_Ms(start_position:standard_size,2);
exp_Ms_qua.y = Go_Ms(start_position:standard_size,3);
exp_Ms_qua.t = (0:T:(standard_size-start_position)*T)';
exp_Ms_qua.name = 'Realimentação M(s)';

exp_Ms_deg.r = Go_degrau(start_position:standard_size,1);
exp_Ms_deg.u = Go_degrau(start_position:standard_size,2);
exp_Ms_deg.y = Go_degrau(start_position:standard_size,3);
exp_Ms_deg.t = (0:T:(standard_size-start_position)*T)';
exp_Ms_deg.name = 'Realimentação M(s)';

%% Leitura dos dados experimento 5 - Controlador avanço atraso

load('exp5.mat');

standard_size = round(30/T);
start_position = round(20/T);

atraso_avanco.r = exp5(start_position:standard_size,1);
atraso_avanco.u = exp5(start_position:standard_size,2);
atraso_avanco.y = exp5(start_position:standard_size,3);
atraso_avanco.t = (0:T:(standard_size-start_position)*T)';
atraso_avanco.name = 'Atraso-Avanço';


%% Leitura dos dados experimento 3 - Controladores anteriores

load('exp3.mat');

% Proporcional
standard_size = round(12/T);
start_position = round(2/T);
proporcional.r = prop_square(start_position:standard_size,1);
proporcional.u = prop_square(start_position:standard_size,2);
proporcional.y = prop_square(start_position:standard_size,3);
proporcional.t = (0:T:(standard_size-start_position)*T)';
proporcional.name = 'Proporcional';

% PID Sisotool
standard_size = round(30/T);
start_position = round(20/T);
pid.r = pid_square(start_position:standard_size,1);
pid.u = pid_square(start_position:standard_size,2);
pid.y = pid_square(start_position:standard_size,3);
pid.t = (0:T:(standard_size-start_position)*T)';
pid.name = 'PID Sisotool';

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

Cu = tf(ss(sys.A-L*sys.C, sys.B, K, 0));
Cy = tf(ss(sys.A-L*sys.C, L, K, 0));

%% Simulação

M0 = (s/tau + 1)/(s/30 + 1);

M = M1;
Ms = M0;
[Ms_num, Ms_den] = tfdata(Ms);

h = K*M;
hs = h*Ms;

%% Comparação com o teórico

teo_M_qua.y = lsim(S*h, exp_M_qua.r, exp_M_qua.t);
teo_M_qua.t = exp_M_qua.t;

teo_Ms_qua.y = lsim(S*hs, exp_Ms_qua.r, exp_Ms_qua.t);
teo_Ms_qua.t = exp_Ms_qua.t;

exp_Ms_ram.r = lsim(1/s, exp_Ms_deg.r, exp_Ms_deg.t);
exp_Ms_ram.y = lsim(1/s, exp_Ms_deg.y, exp_Ms_deg.t);
exp_Ms_ram.t = exp_Ms_deg.t;

t = 0:T:(length(Go_degrau(391:end,3))*T-T);
y_ramp = lsim(1/s, Go_degrau(391:end,3), t);
plot(t,y_ramp)

%% Gráficos

plot_voltage_time_input(exp_M_qua.u, exp_M_qua.t, 'Controlador com M')
plot_voltage_time_input(exp_Ms_qua.u, exp_Ms_qua.t, 'Controlador com M(s)')

plot_voltage_time_compare_reference(exp_M_qua.r, exp_M_qua.y, exp_M_qua.t, 'Controlador com M')
plot_voltage_time_compare_reference(exp_Ms_qua.r, exp_Ms_qua.y, exp_Ms_qua.t, 'Controlador com M(s)')

plot_voltage_time_compare_reference(exp_Ms_ram.r, exp_Ms_ram.y, exp_Ms_ram.t, 'Entrada Rampa')

plot_voltage_time_compare_model(exp_M_qua.y, exp_M_qua.t, teo_M_qua.y, teo_M_qua.t, 'Controlador com M')
plot_voltage_time_compare_model(exp_Ms_qua.y, exp_Ms_qua.t, teo_Ms_qua.y, teo_Ms_qua.t, 'Controlador com M(s)')

data{1} = exp_Ms_qua;
data{2} = atraso_avanco;
data{3} = pid;
data{4} = proporcional;
plot_voltage_time_compare_controller(data, 'Resposta à Onda Quadrada')
plot_voltage_time_compare_controller_input(data, 'Esforço de Controle')


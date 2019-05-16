%% Experimento 05

clear; clc; close all;

[~,~]=mkdir('imagens');

addpath(genpath('dados'))
addpath(genpath('funcoes'))
addpath(genpath('simulink'))

%% Dados importantes
s = tf('s');

load('system.mat');
load('exp3.mat');
load('exp4.mat');
load('exp5.mat');

[num, den] = tfdata(G);
G_num = num{1};
G_den = den{1};

f = 0.5;
ta = 0:1e-6:10;
ra = 0.5*square(2*pi*ta*f) + 0.5;

%% Leitura dos dados experimento 3 - Controladores anteriores

T = 0.002;

% Proporcional
standard_size = round(12/T);
start_position = round(2/T);
proporcional.r = prop_square(start_position:standard_size,1);
proporcional.u = prop_square(start_position:standard_size,2);
proporcional.y = prop_square(start_position:standard_size,3);
proporcional.t = (0:T:(standard_size-start_position)*T)';
proporcional.name = 'Proporcional';

% Ziegler-Nichols
standard_size = round(12/T);
start_position = round(2/T);
ziegler_nichols.r = zn_square(start_position:standard_size,1);
ziegler_nichols.u = zn_square(start_position:standard_size,2);
ziegler_nichols.y = zn_square(start_position:standard_size,3);
ziegler_nichols.t = (0:T:(standard_size-start_position)*T)';
ziegler_nichols.name = 'Ziegler-Nichols';

% PID Sisotool
standard_size = round(30/T);
start_position = round(20/T);
pid.r = pid_square(start_position:standard_size,1);
pid.u = pid_square(start_position:standard_size,2);
pid.y = pid_square(start_position:standard_size,3);
pid.t = (0:T:(standard_size-start_position)*T)';
pid.name = 'PID Sisotool';

%% Leitura dos dados experimento 4 - Controlador analógico

T = 0.002;

standard_size = round(30/T);
start_position = round(20/T);

analogico.r = exp4(start_position:standard_size,1);
analogico.u = exp4(start_position:standard_size,2);
analogico.y = exp4(start_position:standard_size,3)+0.06;
analogico.t = (0:T:(standard_size-start_position)*T)';
analogico.name = 'Analógico';



%% Leitura dos dados experimento 5 - Controlador avanço atraso

T = 0.002;

standard_size = round(30/T);
start_position = round(20/T);

atraso_avanco.r = exp5(start_position:standard_size,1);
atraso_avanco.u = exp5(start_position:standard_size,2);
atraso_avanco.y = exp5(start_position:standard_size,3);
atraso_avanco.t = (0:T:(standard_size-start_position)*T)';
atraso_avanco.name = 'Atraso-Avanço';

%% Atraso Avanço

K = 3;
Md = 42;

[~,Mf,~,~] = margin(K*G);

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

Fvt = Cvt*G/(1+Cvt*G);

y = lsim(Fvt, atraso_avanco.r, atraso_avanco.t);
u = lsim(Cvt/(1+Cvt*G), atraso_avanco.r, atraso_avanco.t);

modelo_vt.y = y;
modelo_vt.u = u;
modelo_vt.r = atraso_avanco.r;
modelo_vt.t = atraso_avanco.t;
modelo_vt.name = 'Teórico';
%%

plot_voltage_time_compare_reference(atraso_avanco.r, atraso_avanco.y, atraso_avanco.t, 'Controlador Experimental')
plot_voltage_time_input(atraso_avanco.u, atraso_avanco.t, 'Controlador Experimental')

atraso_avanco.name = 'Experimental';

data{1} = atraso_avanco;
data{2} = modelo_vt;

plot_voltage_time_compare_controller(data, ' Experimental x Teórico')

atraso_avanco.name = 'Atraso-Avanço';
data{1} = atraso_avanco;
data{2} = analogico;
data{3} = pid;
plot_voltage_time_compare_controller(data, ' Controladores Experimentais')

data{2} = proporcional;
data{3} = ziegler_nichols;
plot_voltage_time_compare_controller(data, ' Demais Controladores')
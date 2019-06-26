%% Experimento 08

clear; clc; close all;

[~,~]=mkdir('imagens');

addpath(genpath('dados'))
addpath(genpath('funcoes'))
addpath(genpath('simulink'))

%% Leitura dos dados

load('julio_pendulo.mat')
exp_pendulo.t = data(1,:);
exp_pendulo.theta = data(2,:);

load('julio_carro_3V.mat')
exp_carro_3V.t = data(1,:);
exp_carro_3V.v = data(2,:);

load('julio_carro_4V.mat')
exp_carro_4V.t = data(1,:);
exp_carro_4V.v = data(2,:);

load('turmaB_Mk_3V.mat')
exp_Mk_3V.t = data(1,:);
exp_Mk_3V.v = data(2,:);

load('turmaB_Mk_4V.mat')
exp_Mk_4V.t = data(1,:);
exp_Mk_4V.v = data(2,:);

%% Dados já conhecidos

Mp = 0.23; % [kg]
Mk = 0.5;  % [kg]
Ip = 0.3302;
g = 9.81;  % [m/s^2]

%% Carro

% plot(exp_carro_3V.t, exp_carro_4V.v)
% plot(exp_carro_4V.v)

v_regime(1) = mean(exp_carro_3V.v(end-700:end));
v_regime(2) = mean(exp_carro_4V.v(end-700:end)); % Tinha uma rampa no final

k(1) = v_regime(1)/3;
k(2) = v_regime(2)/4;

k0 = mean(k);

v_tau = 0.63*v_regime;

testes = [exp_carro_3V exp_carro_4V];

for i=1:length(testes)
    
    exp = testes(i);
    loc = find(exp.v >= v_tau(i), 1);

    ini = max(1, loc-3);
    fim = ini+7;
    tau_testes(i) = interp1(exp.v(ini:fim), exp.t(ini:fim), v_tau(i));
end

tau = mean(tau_testes) - 1;

%% Massa adicional

% plot(exp_Mk_3V.v)

v_regime_k(1) = mean(exp_Mk_3V.v(end-700:end));
v_regime_k(2) = mean(exp_Mk_4V.v(end-700:end)); % Tinha uma rampa no final


% v_regime_k(1) = max(exp_Mk_3V.v);
% v_regime_k(2) = max(exp_Mk_4V.v); % Tinha uma rampa no final

k_k(1) = v_regime_k(1)/3;
k_k(2) = v_regime_k(2)/4;

k0_k = mean(k_k);

v_tau_k = 0.63*v_regime_k;

testes = [exp_Mk_3V exp_Mk_4V];

for i=1:length(testes)
    
    exp = testes(i);
    loc = find(exp.v >= v_tau_k(i), 1);

    ini = max(1, loc-3);
    fim = ini+7;
    tau_testes_k(i) = interp1(exp.v(ini:fim), exp.t(ini:fim), v_tau_k(i));
end

tau_k = mean(tau_testes_k) - 1;

% Verificar se os tau são iguais

Beq = Mk/(tau_k-tau);
Meq = Beq*tau;
Aeq = Beq*k0;

%% Pêndulo

dt = exp_pendulo.t(2) - exp_pendulo.t(1);
T = exp_pendulo.t(end);

f = 0:1/T:1/dt;

pendulo_fft =  abs(fft(exp_pendulo.theta));

% plot(f, pendulo_fft);

[~, f_i] = max(pendulo_fft(1:end/2));
Fhz = f(f_i);

wd = 2*pi*Fhz;


abs_angle = abs(exp_pendulo.theta);
time = exp_pendulo.t(1:length(abs_angle));

% identificacao da freq natural e do coef amortecimento
[pks, loc] = findpeaks(abs_angle, time);
peaks = pks(10:25);
locs = loc(10:25);

m = length(peaks);

sum_loc = 0;
sum_ang = 0;
for i = 1:m-1
    sum_loc = sum_loc+(locs(i+1)-locs(i));
    sum_ang = sum_ang+(log(peaks(i)) - log(peaks(i+1)));
end

% tangente de phi
tg_phi = (m-1)*pi/sum_ang;
% coef de amortecimento
qsi = sqrt(1/(tg_phi^2 + 1));
% freq natural
wn = wd/(qsi*tg_phi);

%%

Jp = Mp * Ip * g/(wn^2) - Mp*Ip^2;
Bp = 2*qsi*wn*(Mp*Ip^2+Jp);

%% Sistema

Jt = Meq*(Mp*Ip^2 + Jp) + Jp*Mp;

A = 1/Jt* [
    0 0 Jt 0
    0 0 0 Jt
    0 (Mp^2)*(Ip^2)*g  -Beq*(Mp*Ip^2 + Jp)   -Bp*Mp*Ip
    0 Mp*Ip*g*(Meq+Mp) -Beq*Mp*Ip          -Bp*(Meq+Mp)
];

B = Aeq/Jt * [
    0
    0
    Mp*(Ip^2)+Jp
    Mp*Ip
];

C = eye(4);

D = 0;

sys = ss(A, B, C, D);

s = tf('s');
G = tf(sys);

%% Controle

[num, den] = tfdata(G);
G_num = num{1};
G_den = den{1};

V = [1 1 0 0];

phi = tf(ss(A, B, V, 0));

Gs = phi'*phi;

%sisotool(phi'*phi,rho^-1);

rho = 1/818.66;

[K, ~, E] = lqr(sys, V'*V, rho);

% pole = min(real(E));
% 
% po = ones(size(E))*pole*4;
% L = acker(A', C', po)';
% 
% Cu = tf(ss(A-L*C, B, K, 0));
% Cy = tf(ss(A-L*C, L, K, 0));

fprintf("K = \\begin{bmatrix} %f & %f & %f & %f \\end{bmatrix}\n", K(1), K(2), K(3), K(4))


%% Segunda Parte - Simulação

x0 = [0 pi/18 0 0]';

SquareWave = 0;
sim('state_feedback_controller_no_observer.slx', 10)
sim_data.t = y(:,1);
sim_data.y = y(:,2);
sim_data.x = x(:,2:end);
sim_data.u = u(:,2);
sim_data.name = "Alocação de Polos";

plot_voltage_time_input(sim_data.u, sim_data.t, 'Controlador')


name = 'Posição - Controlador';
figure;
subplot(2, 1, 1)
plot(sim_data.t, sim_data.x(:,1))
ylabel('x_c [m]')
xlabel('t [s]')
title('Deslocamento')

subplot(2, 1, 2)
plot(sim_data.t, sim_data.x(:,2))
ylabel('\theta [rad]')
xlabel('t [s]')
title('Posição ângular')

name = removeSpecialCharacters(name);
saveas(gcf, strcat('imagens/', name, '.png'));


%% Validação da planta - Pendulo


t_pen_1 = exp_pendulo.t;
u_pen_1 = zeros(size(t_pen_1));

alpha_0 = max(pks);
alpha = -alpha_0*(s + 2*qsi*wn)/(s^2+2*qsi*wn*s + wn^2);

y_pen_1 = impulse(alpha, t_pen_1);


c = 2631;

figure
hold all
plot(t_pen_1, y_pen_1);
plot(exp_pendulo.t(1:end-c+1), exp_pendulo.theta(c:end));
xlabel('Tempo [s]')
ylabel('\theta [rad]')
title('Validação do modelo')
legend('Modelo', 'Experimento')

%% Validação da planta - Carrinho

t_car_1 = exp_carro_3V.t;
u_car_1 = zeros(size(t_car_1));

carrinho = k0/(tau*s + 1);

y_car_1 = 3*step(carrinho, t_car_1);

c = 1e3;

figure
hold all
plot(t_car_1, y_car_1);
plot(exp_carro_3V.t(1:end-c+1), exp_carro_3V.v(c:end));
xlabel('Tempo [s]')
ylabel('\theta [rad]')
title('Validação do modelo')
legend('Modelo', 'Experimento')


%% Validação da planta - Carrinho

t_car_1 = exp_carro_4V.t;
u_car_1 = zeros(size(t_car_1));

carrinho = k0/(tau*s + 1);

y_car_1 = 4*step(carrinho, t_car_1);

c = 1e3;

figure
hold all
plot(t_car_1, y_car_1);
plot(exp_carro_4V.t(1:end-c+1), exp_carro_4V.v(c:end));
xlabel('Tempo [s]')
ylabel('\theta [rad]')
title('Validação do modelo')
legend('Modelo', 'Experimento')
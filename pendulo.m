close all; clear all; clc;

% carrega os dados do pendulo
load('rafael_pendulo.mat');
time = data(1,:)';
angle = data(2,:)';
% Remove o inicio da medicao ruim
c = 3360;
dt = time(c);
time = time(c:end);
time = time-dt;
angle = angle(c:end);

% calcula os valores absolutos dos angulos para considerar os minimos
abs_angle = abs(angle);

% parametros do pendulo
% massa
Mp = 0.23;
% comprimento
lp = 0.3302;
% gravidade
g = 9.81;

% dinamica do pendulo
% plot(time, angle)

% identificacao da freq natural e do coef amortecimento
[pks, loc] = findpeaks(abs_angle, time);
peaks = pks(1:15);
locs = loc(1:15);

m = length(peaks);

sum_loc = 0;
sum_ang = 0;
for i = 1:m-1
    sum_loc = sum_loc+(locs(i+1)-locs(i));
    sum_ang = sum_ang+(log(peaks(i)) - log(peaks(i+1)));
end

% Frequencia amortecida
wd = (m-1)*pi/sum_loc;
% tangente de phi
tg_phi = (m-1)*pi/sum_ang;
% coef de amortecimento
xi = sqrt(1/(tg_phi^2 + 1));
% freq natural
wn = wd/(xi*tg_phi);

% momento de inercia
Jp = Mp*lp*g/wn^2 - Mp*lp^2;
% coeficiente de atrito viscoso
Bp = 2*xi*wn*(Mp*lp^2 + Jp);

% % Validacao
% s = tf('s');
% Alpha = peaks(1)*(s+2*xi*wn)/(s^2+2*xi*wn*s+wn^2);
% y = impulse(Alpha, time);
% 
% figure;
% plot(time,y);
% hold on;
% plot(time, angle, 'r');
%% Outro metodo:

% neg_angle = -angle;
% 
% [pos_pks, pos_loc] = findpeaks(angle, time);
% [neg_pks, neg_loc] = findpeaks(neg_angle, time);
% neg_pks = -neg_pks;
% 
% % figure;
% % plot(pos_loc, pos_pks, 'o');
% % hold on;
% % plot(neg_loc, neg_pks, 'o');
% 
% % observando os pontos plotados, resolvi usar os maxs e mins entre os
% % indices 5 e 80 para tentar desconsiderar os efeitos do atrito seco.
% 
% pos_pks = pos_pks(1:30);
% pos_loc = pos_loc(1:30);
% neg_pks = neg_pks(1:30);
% neg_loc = neg_loc(1:30);
% 
% % concatena os arrays para considerar maximos e minimos
% locs = sort(cat(1, pos_loc, neg_loc));
% % concatena os valores dos angulos (absolutos)
% peaks = sort(abs(cat(1, pos_pks, neg_pks)), 'descend');
% 
% m = length(peaks);
% 
% sum_loc = 0;
% sum_ang = 0;
% for i = 1:m-1
%     sum_loc = sum_loc+(locs(i+1)-locs(i));
%     sum_ang = sum_ang+(log(peaks(i)) - log(peaks(i+1)));
% end
% 
% % Frequencia amortecida
% wd = (m-1)*pi/sum_loc;
% % tangente de phi
% tg_phi = (m-1)*pi/sum_ang;
% % coef de amortecimento
% xi = sqrt(1/(tg_phi^2 + 1));
% % freq natural
% wn = wd/(xi*tg_phi);
% 
% % momento de inercia
% Jp = Mp*lp*g/wn^2 - Mp*lp^2;
% % coeficiente de atrito viscoso
% Bp = 2*xi*wn*(Mp*lp^2 + Jp);

%% Parametros do carro

Mk = 0.5;

% Sem a massa extra
% Tensoes em regime (medias)
Vr_3 = 0.3984;
Vr_4 = 0.5454;
% Taus
Vtau_3 = 0.63*Vr_3;
tau_3 = 0.213;

Vtau_4 = 0.63*0.5454;
tau_4 = 0.219;

% Com a massa extra
% tensoes em regime
Vr_3m = 0.3380;
Vr_4m = 0.5131;
% Taus
Vtau_3m = 0.63*Vr_3m;
tau_3m = 0.307;

Vtau_4m = 0.63*Vr_4m;
tau_4m = 0.284;

a = (1-exp(-1));

% Tau e Kappa
tau = mean([tau_3 tau_4]);
K0 = mean([Vtau_3/(3*a) Vtau_4/(4*a)]);

tau_m = mean([tau_3m tau_4m]);
K0_m = mean([Vtau_3m/(3*a) Vtau_4m/(4*a)]);

% Parametros do carrinho
Beq = Mk/(tau_m - tau);
Meq = Beq*tau;
Aeq = Beq*K0;

%% Modelo em espaco de estado

Jt = Meq*(Mp*lp^2+Jp)+Jp*Mp;

A = (1/Jt)*[0 0 Jt 0;
            0 0 0 Jt;
            0 Mp^2*lp^2*g -Beq*(Mp*lp^2+Jp) -Bp*Mp*lp;
            0 Mp*lp*g*(Meq+Mp) -Beq*Mp*lp -Bp*(Meq+Mp)];

B = (Aeq/Jt)*[0; 0; (Mp*lp^2+Jp); Mp*lp];

C = eye(4);

D = zeros(4,1);

%% Controlador LQR

% Matriz V - pesos iguais para x e theta
V = [1 0 0 0; 0 1 0 0];
Q = V'*V;
% rho ajustado empiricamente
rho = 1e-3;

% Espaco de estado do sistema
sys = ss(A,B,C,D);

% Controlador LQR
[K,P,E] = lqr(sys,Q,rho);

% Funcao de transferencia em malha fechada
G = ss(A-B*K,B,C,D);
% Condicoes iniciais
X0 = [0 10*pi/180 0 0];
% Tempo de simulacao
T = 0:0.01:5;
% Referencia do sistema
U = zeros(1,length(T));
% Simulacao
res = lsim(G, U, T, X0);
% esforco de controle
efc = -K*res';

figure;
plot(T, res(:,1));
title("Posição do carro.");
xlabel("Tempo [s]");
ylabel("Posição [m]");
set(gca,'FontSize',20);

figure;
plot(T,efc);
title("Esforço de Controle. u_{min}: "+min(efc));
xlabel("Tempo [s]");
ylabel("Tensão [V]");
set(gca,'FontSize',20);

figure;
plot(T,res(:,2));
title("Posição angular do pêndulo.");
xlabel("Tempo [s]");
ylabel("Ângulo [rad]");
set(gca,'FontSize',20);

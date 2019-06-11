%% Experimento 08

clear; clc; close all;

[~,~]=mkdir('imagens');

addpath(genpath('dados'))
addpath(genpath('funcoes'))
addpath(genpath('simulink'))

%% Leitura dos dados

load('parado.csv')
% load('parado2.csv')
% parado = parado2;

load('d10.csv')
load('d12.csv')
load('d14.csv')
load('d16.csv')

dt = 10e-3;

m10.w = d10(1,:);
m10.i = d10(2,:);
m10.t = 0:dt:(length(d10)-1)*dt;
m10.v = 10;

m12.w = d12(1,:);
m12.i = d12(2,:);
m12.t = 0:dt:(length(d12)-1)*dt;
m12.v = 12;

m14.w = d14(1,:);
m14.i = d14(2,:);
m14.t = 0:dt:(length(d14)-1)*dt;
m14.v = 14;

m16.w = d16(1,:);
m16.i = d16(2,:);
m16.t = 0:dt:(length(d16)-1)*dt;
m16.v = 16;

mparado.w = parado(1,:);
mparado.i = parado(2,:);
mparado.t = 0:dt:(length(parado)-1)*dt;
mparado.v = 14;

%% Motor parado - R, L

i_regime = mean(mparado.i(end-50:end));
i_tau = 0.63*i_regime;

loc = find(mparado.i >= i_tau, 1);


ini = max(1, loc-3);
fim = ini+7;
tau = interp1(mparado.i(ini:fim), mparado.t(ini:fim), i_tau);

% tau = tau - mparado.t(314);

motor.R = 14/i_regime;
motor.L = tau*motor.R;

%% Motor em movimento - K, b, Tc

mds = [m10 m12 m14 m16];

regime = [1453 1540 2848 1718];

for k=1:4
    md = mds(k);
    fim_regime = regime(k);

    i_regime = mean(md.i(fim_regime-100:fim_regime));
    w_regime = mean(md.w(fim_regime-100:fim_regime));
    
    K(k) = (md.v-motor.R*i_regime)/w_regime;
    
    T(k) = K(k)*i_regime;
    w(k) = w_regime;

end

motor.K = mean(K);

motor.Tc = interp1(w, T, 0, 'linear', 'extrap');

motor.b = (T(1) - motor.Tc)/w(1);

fnc_T = @(w) (motor.Tc + motor.b*w);
w_T = [0 w];

name = 'Relação Torque x Rotação';
figure
hold on
plot(w_T, fnc_T(w_T), '--')
plot(w, T)
hold off
ylabel('T [Nm]')
xlabel('v [rad/s]')
legend('Interpolado', 'Original')
title(name)

name = removeSpecialCharacters(name);
saveas(gcf, strcat('imagens/', name, '.png'));

%% Mecânica do motor - J

wtj = (w + motor.Tc/motor.b).*exp(-1/4) - motor.Tc/motor.b;


mds = [m10 m12 m14 m16];

regime = [1458 1544 2865 1802];

for k=1:4
    md = mds(k);
    loc = regime(k)-1 + find(md.w(regime(k):end) <= wtj(k), 1);

    tjs(k) = md.t(loc) - md.t(regime(k)-1);
end

tj = mean(tjs);

motor.J = 4*motor.b*tj;

%% Modelo

% x = [i w theta]'
% u = [V Tc]'

A = [
    -motor.R/motor.L   -motor.K/motor.L   0
     motor.K/motor.J   -motor.b/motor.J   0
     0                  1                 0
];

B = [
     1/motor.L   0
     0          -1/motor.J
     0           0
];

C = [
    1 0 0
    0 1 0 
];

D = 0;

sys = ss(A, B, C, D);
Gm = tf(sys);
G = Gm(2,1);

%% Simulação

for k = 1:4
    md = mds(k);
    fim = regime(k);
    
    t = md.t(1:fim);
    lt = size(t);

    u = [
        md.v*ones(lt) 
        motor.Tc*ones(lt)
    ];

    Y = lsim(sys, u, t);
    y_i = Y(:,1);
    y_w = Y(:,2);
    
    v = [int2str(md.v) 'v'];
    
    figure;
    subplot(2, 1, 1)
    hold on
    plot(t, md.w(1:fim))
    plot(t, y_w)
    hold off
    ylabel('v [rad/s]')
    xlabel('t [s]')
    title(['Modelo x Experimento - ' v ' - Rotação'])
    legend('Modelo', 'Experimento')
    
    subplot(2, 1, 2)
    hold on
    plot(t, md.i(1:fim))
    plot(t, y_i)
    hold off;
    ylabel('i [amp]')
    xlabel('t [s]')
    title(['Modelo x Experimento - ' v ' - Corrente'])
    legend('Modelo', 'Experimento')
    
    name = ['Comparação Modelo x Experimento - ' v];
    name = removeSpecialCharacters(name);
    saveas(gcf, strcat('imagens/', name, '.png'));
end

%% Dados experimentais


for k = 1:4
    md = mds(k);
    
    v = [int2str(md.v) 'v'];
    
    figure;
    subplot(2, 1, 1)
    hold on
    plot(md.t, md.w)
    hold off
    ylabel('v [rad/s]')
    xlabel('t [s]')
    title(['Dados Experimentais - ' v ' - Rotação'])
    
    subplot(2, 1, 2)
    hold on
    plot(md.t, md.i)
    hold off;
    ylabel('i [amp]')
    xlabel('t [s]')
    title(['Dados Experimentais - ' v ' - Corrente'])
    
    name = ['Dados Experimentais - ' v];
    name = removeSpecialCharacters(name);
    saveas(gcf, strcat('imagens/', name, '.png'));
end
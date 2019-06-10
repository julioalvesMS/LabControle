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

rhos = [1e-3 1e-2 1e-1 1];

clear data;
for rho=rhos
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
    
    
    x0 = [0 0 0]';
    Ms_num{1} = 1; Ms_den{1} = 1;
    
    SquareWave = 1;
    sim('state_feedback_controller.slx', 0.15)

    degrau.t = y(:,1);
    degrau.y = y(:,2);
    degrau.r = r(:,2);
    degrau.u = u(:,2);
    
    degrau.K = K;
    degrau.L = L;
    degrau.rho = rho;
    degrau.Q = Q;
    degrau.M = M;
    degrau.name = sprintf("\\rho = %.3f", rho);
    
    degrau.Cu = tf(ss(A-L*C, B, K, 0));
    degrau.Cy = tf(ss(A-L*C, L, K, 0));
    
    exist data var;
    if ans ~= 0
        data{end+1} = degrau;
    else
        data{1} = degrau;
    end
end

%% Primeira Parte - Gráficos

plot_voltage_time_compare_controller(data, 'Progressão do \rho')
plot_voltage_time_compare_controller_input(data, 'Progressão do \rho')


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

Cu = tf(ss(A-L*C, B, K_lqr, 0));
Cy = tf(ss(A-L*C, L, K_lqr, 0));


fprintf("K = \\begin{bmatrix} %f & %f & %f \\end{bmatrix}\n", K_cal(1), K_cal(2), K_cal(3))
fprintf("K = \\begin{bmatrix} %f & %f & %f \\end{bmatrix}\n", K_lqr(1), K_lqr(2), K_lqr(3))

%% Segunda Parte - Simulação

x0 = [0 0 0]';
Ms_num{1} = 1; Ms_den{1} = 1;
K = K_cal;

SquareWave = 1;
sim('state_feedback_controller.slx', 1)
degrau.t = y(:,1);
degrau.y = y(:,2);
degrau.r = r(:,2);
degrau.u = u(:,2);
degrau.name = "Alocação de Polos";



plot_voltage_time(degrau.y, degrau.t, 'Controlador via Lugar das Raízes')
plot_voltage_time_input(degrau.u, degrau.t, 'Controlador via Lugar das Raízes')


%%

for control=data
    fprintf("\n rho = %.4f \n", control{1}.rho)
    fprintf("K = \\begin{bmatrix} %f & %f & %f \\end{bmatrix}\n", control{1}.K(1), control{1}.K(2), control{1}.K(3))
    fprintf("L = \\begin{bmatrix} %f \\\\ %f \\\\ %f \\end{bmatrix}\n", control{1}.L(1), control{1}.L(2), control{1}.L(3))
    fprintf("M = \\begin{bmatrix} %.10f \\\\ %.0f \\\\ %.0f \\end{bmatrix}\n", control{1}.M(1), control{1}.M(2), control{1}.M(3))
end
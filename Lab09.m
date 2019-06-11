%% Experimento 08

clear; clc; close all;

[~,~]=mkdir('imagens');

addpath(genpath('dados'))
addpath(genpath('funcoes'))
addpath(genpath('simulink'))

%% Leitura dos dados

load('dados_motor.mat')

Vt = [20 50 80];
s = tf('s');
x0 = [0 0 0]';

%% Proporcional - Projeto 1

%sisotool(G, 11.867)

Cp1 = 11.867;

C_num = Cp1;
C_den = 1;

T = 0.2;

p1_data = {};
for i=1:length(Vt)
    v = Vt(i);
    
    sim('motor_output_feedback.slx', T)
    p1_data.u{i} = u(:,2);
    p1_data.y{i} = y(:,3);
    p1_data.t{i} = y(:,1);
    
    p1_data.legend{i} = [int2str(v) ' rad/s'];
end
p1_data.name = 'Prorcional - Projeto 1';

plot_rotation_control(p1_data)

%% Proporcional - Projeto 2

%sisotool(G, 0.015111)

Cp2 = 0.015111;
C_num = Cp2;
C_den = 1;

T = 30;

p2_data = {};
for i=1:length(Vt)
    v = Vt(i);
    
    sim('motor_output_feedback.slx', T)
    p2_data.u{i} = u(:,2);
    p2_data.y{i} = y(:,3);
    p2_data.t{i} = y(:,1);
    
    p2_data.legend{i} = [int2str(v) ' rad/s'];
end
p2_data.name = 'Prorcional - Projeto 2';

plot_rotation_control(p2_data)


%% Integrador - Projeto 1

%sisotool(G, 0.005779/s)

Ci1 = 0.005779/s;
[C_num, C_den] = tfdata(Ci1, 'v');

T = 80;

i1_data = {};
for i=1:length(Vt)
    v = Vt(i);
    
    sim('motor_output_feedback.slx', T)
    i1_data.u{i} = u(:,2);
    i1_data.y{i} = y(:,3);
    i1_data.t{i} = y(:,1);
    
    i1_data.legend{i} = [int2str(v) ' rad/s'];
end

i1_data.name = 'Integrador - Projeto 1';

plot_rotation_control(i1_data)

%% Proporcional Integrador - Projeto 1

%sisotool(G, (0.027127 *(s+0.4757))/s)

kp = 0.0271;
ki = 0.0129;
Cpi1 = kp + ki/s;
[C_num, C_den] = tfdata(Cpi1, 'v');

T = 30;

cpi1_data = {};
for i=1:length(Vt)
    v = Vt(i);
    
    sim('motor_output_feedback.slx', T)
    cpi1_data.u{i} = u(:,2);
    cpi1_data.y{i} = y(:,3);
    cpi1_data.t{i} = y(:,1);
    
    cpi1_data.legend{i} = [int2str(v) ' rad/s'];
end

cpi1_data.name = 'Proporcional Integrador - Projeto 1';

plot_rotation_control(cpi1_data)

clear; clc; close all;

[~,~]=mkdir('imagens');

addpath(genpath('dados'))

%% Planta a ser controlada
s = tf('s');

load('system.mat');

%% Análises

sisotool(G)
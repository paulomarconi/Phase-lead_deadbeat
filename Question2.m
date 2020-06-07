%% ========================================================================
% ---- ACS6101 Assignment week 5
% ---- Registration number: 180123717
% ---- Name: Paulo Roberto Loma Marconi
% ---- 03/11/2018
%% === Question 2 =========================================================
clear; clc; close all;
s = tf('s');
Gp = 0.04*(s+1)/(s^2+0.2*s+0.04); % uncompensated plant
Ts = 1; % sampling time
Gz = c2d(Gp,Ts,'zoh'); % System transfer function in discrete time
% Minimum phase case
z=zpk('z',Ts);
Mz = 1/z; % closed-loop because the relative order d = 1
Dz = Mz/(Gz*(1-Mz)); % deadbeat controller
Dz = minreal(Dz); % cancel common factors
fig = figure(1); 
step(Dz*Gz/(1+Dz*Gz)) % step response of closed-loop
saveas(fig,'Q2_deadbeat_step.png');
fig = figure(2); 
step(Dz/(1+Dz*Gz)) % control signal
title('Control signal');
saveas(fig,'Q2_deadbeat_control.png');
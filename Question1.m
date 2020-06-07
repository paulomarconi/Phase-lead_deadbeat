%% ========================================================================
% ---- ACS6101 Assignment week 5
% ---- Registration number: 180123717
% ---- Name: Paulo Roberto Loma Marconi
% ---- 03/11/2018
%% === Question 1 =========================================================
clear; clc; close all; % clean previous data in order to avoid erros.
%% plant Gp 
s = tf('s');
Gp = 0.04*(s+1)/(s^2+0.2*s+0.04); % uncompensated plant

fig = figure(1); 
margin(Gp); % calculates the phase margin and gain margin at their frequencies
[Gm,Pm,Wcg,Wcp] = margin(Gp);
Gcl = feedback(Gp,1); % closed-loop of the uncompensated plant
saveas(fig,'Q1_Gp_margin.png');

fig = figure(2);
step(Gcl); % step response to the closed-loop system
stepinfo(Gcl) % system performance values
saveas(fig,'Q1_Gp_step.png');
%% Design requirements
PO = 10; % percentage overshoot
zeta = log(100/PO)/sqrt(pi^2+ (log(100/PO))^2 ); % damping ratio
PM_d = round(100*zeta)+1; % desired PM
%% Obtaining the gain K that meets the desired PM_d
K = 10^(8.03/20); % the gain 8.03 obtained from the bode plot
Gp1 = K*Gp; % new uncompensated plant

fig = figure(11);
margin(Gp1);
saveas(fig,'Q1_Gp1_K_margin.png');

%% Digital uncompensated system with the new gain K
Ts = 0.01; % sampling time
Gz1 = c2d(Gp1,Ts,'zoh'); % convert the continuos time plant Gp2 to the
                         % discrete time domain with the zero order holder
Gp1_cl = feedback(Gp1,1); % closed-loop system in continuos time
Gz1_cl = feedback(Gz1,1); % closed-loop system in discrete time

fig = figure(3);
step(Gp1_cl,Gz1_cl); % step response of both continuos and discrete systems
saveas(fig,'Q1_Gpz_K_step.png');
stepinfo(Gz1_cl) % system performance values

%% Designing the Phase-lead digital controller 
% introducing a new gain 10 times faster in ordert to obtain a fast
% response to the step input
Gp2 = K*10*Gp;

fig = figure(4);
margin(Gp2); % checking the desired phase margin PM_d 
[Gm1,PM_act,Wcg1,Wcp1] = margin(Gp2);
saveas(fig,'Q1_Gp2_margin.png');

% step 1) obtaining beta with the actual PM and the desired PM
theta = 6; % correction factor
beta = roots( [1 -(2+4*( tand(PM_d-PM_act+theta) )^2) 1] );
% beta = (1+sind(PM_d-PM_act+theta))/(1-sind(PM_d-PM_act+theta));
% step 2) calculating the new crossover frequency
Mpc = 1/sqrt(beta(1)); % find compensator peak magnitude.
omega_c = getGainCrossover(Gp2,Mpc); % The new gain crossover frequency wc
% step 3) determining the zero and the pole of the controller
zc = omega_c/sqrt(beta(1)); % zero of the controller
pc = beta(1)*zc; % pole of the controller
% step 4) controller in continuous and discrete time
Gc = beta(1)*(s+zc)/(s+pc); % phase-lead controller in continuous time
Gcz = c2d(Gc,Ts,'zoh'); % phase-lead controller in discrete time
%% Evaluating the phase-lead controller
Gol = Gc*Gp2; % open-loop compensated in continuous time
Gcl = feedback(Gol,1); % closed-loop in continuous time
Gp2z = c2d(Gp2,Ts,'zoh'); % plant in discrete time
Golz = Gcz*Gp2z; % open-loop compensated in discrete time
Gclz = feedback(Golz,1); % closed-loop in discrete time

fig = figure(5);
margin(Gol);
saveas(fig,'Q1_lead_margin.png');

fig = figure(6);
step(Gcl,Gclz);
saveas(fig,'Q1_lead_step.png');

%% Designing a second lead compensator
[Gm11,PM_act1,Wcg11,Wcp11] = margin(Gol);
% step 1) obtaining beta with the actual PM and the desired PM
theta1 = 6; % correction factor
beta1 = roots( [1 -(2+4*( tand(PM_d-PM_act1+theta1) )^2) 1] );
% beta1 = (1+sind(PM_d-PM_act+theta))/(1-sind(PM_d-PM_act+theta));
% step 2) calculating the new crossover frequency
Mpc1 = 1/sqrt(beta1(1)); % find compensator peak magnitude.
omega_c1 = getGainCrossover(Gol,Mpc1); % The new gain crossover frequency wc
% step 3) determining the zero and the pole of the controller
zc1 = omega_c1/sqrt(beta1(1)); % zero of the controller
pc1 = beta1(1)*zc1; % pole of the controller
% step 4) controller in continuous and discrete time
Gc1 = beta1(1)*(s+zc1)/(s+pc1); % phase-lead controller in continuous time
Gcz1 = c2d(Gc1,Ts,'zoh'); % phase-lead controller in discrete time
%% Evaluating second the phase-lead controller in series with the previous one
Gol_1 = Gc1*Gc*Gp2; % open-loop compensated in continuous time
Gcl_1 = feedback(Gol_1,1); % closed-loop in continuous time
Golz_1 = Gcz*Gp2z; % open-loop compensated in discrete time
Gclz_1 = feedback(Golz_1,1); % closed-loop in discrete time

fig = figure(7);
margin(Gol_1);
saveas(fig,'Q1_lead_lead_margin.png');

fig = figure(8);
step(Gcl_1,Gclz_1);
saveas(fig,'Q1_lead_lead_step.png');

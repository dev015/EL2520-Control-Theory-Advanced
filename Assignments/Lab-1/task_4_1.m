%% Lead-Lag Compensation
clear; clc; close all;
num = [-3,3];
den = [50,15,1];
G = tf(num,den);
%% Error Reduction
[mag_uc,~,~] = bode(G,0.4);
K = 1/mag_uc;
num_1 = [-3*K, 3*K];
G1 = tf(num_1,den);
%margin(G1)
% Lead Compensation
w_d = 0.4;
[mag_gc,~,~] = bode(G1,w_d);
[Gm_uc,Pm_uc,Wcg_uc,Wcp_uc] = margin(G1);
Pm_d = 50;
%Sf = 1.2;% for Pm_d = 30
Sf = 2.8;
Phi_max = Pm_d + Sf - Pm_uc;
beta = (1 - sind(Phi_max))./(1 + sind(Phi_max));
tau_d = 1/(w_d * (sqrt(beta)));
% TF for Lead COmpensator
num_lead = [K*tau_d, K];
den_lead = [beta*tau_d, 1];
F_lead = tf(num_lead,den_lead);
% Bode for OL Plant with Lead Compensator
%margin(G*F_lead)
[mag_lead,~,~] = bode(G*F_lead,0.4);
mag_lead_dB = 20*log10(mag_lead);
alpha = 10^(mag_lead_dB/20);
tau_i = 10/w_d;
F_lag = tf([tau_i, 1],[alpha*tau_i, 1]);
margin(G*F_lead*F_lag)
%% Closed loop System
Gcl = feedback(G*F_lead*F_lag,1);
Mt = 20*log10(getPeakGain(Gcl));
figure
margin(Gcl)
figure
stepplot(Gcl)
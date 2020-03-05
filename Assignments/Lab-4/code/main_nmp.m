%% Lab4 - Decoupling and Glover-McFarlane Robust Loop Shaping
clc;clear;close all;
s = tf('s');
%% Loading the data
G = nonminphase;
%[G_num,G_den] = tfdata(G_ss);
%G = tf(G_num,G_den);
G0_nmp = dcgain(G);
%% DYNAMIC DECOUPLING
%% Exercise: 3.2.1
RGA = G0_nmp .* inv(G0_nmp)';
W2 = eye(2,2);
g11 = G(1,1);
g12 = G(1,2);
g21 = G(2,1);
g22 = G(2,2);

W1 = [-g22/g21,1; 1,-g11/g12];
W1 = minreal(W1*(0.2/(s+0.2)));
G_tilde = minreal(W2*G*W1);
figure
bode(G_tilde)
hold on 
grid on
hold off
%% Exercise: 3.2.2
% Designing diagonal PI controllers
PM_d = pi*5/18;
Wc_d = 0.1;
% F11
gt11 = G_tilde(1,1);
[Kp1,Ki1] = PIcontrol(PM_d,Wc_d,gt11);
F11 = Kp1*(1+(Ki1/s));
% F22
gt22 = G_tilde(2,2);
[Kp2,Ki2] = PIcontrol(PM_d,Wc_d,gt22);
F22 = Kp2*(1+(Ki2/s));
% F_tilde
F_tilde = [F11,0;0,F22];
%% Controller F(s)
F = minreal(W1*F_tilde);
%% Exercise: 3.2.3
% S = minreal(1/(eye(2,2)+(G*F_old)));
% T = minreal(S*G*F_old);
% [sv_S,w_S] = sigma(S);
% [sv_T,w_T] = sigma(T);
% [sv_S1_max,id_S1] = max(sv_S(1,:));
% [sv_S2_max,id_S2] = max(sv_S(2,:));
% [sv_T1_max,id_T1] = max(sv_T(1,:));
% [sv_T2_max,id_T2] = max(sv_T(2,:));
%% Glover-McFarlane Robust Loop Shaping
% alpha = 1.1;
% L = minreal(G*F_old);
% [Fr,gam] = rloop(L,alpha);
% F = minreal(F_old * Fr);
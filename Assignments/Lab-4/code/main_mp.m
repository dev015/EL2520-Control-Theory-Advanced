%% Lab4 - Decoupling and Glover-McFarlane Robust Loop Shaping
clc;clear;close all;
s = tf('s');
%% Loading the data
G_ss = minphase;
[G_num,G_den] = tfdata(G_ss);
G = tf(G_num,G_den);
G0_mp = dcgain(G);
%% Exercise: 3.1.1
W1 = inv(G0_mp);
W2 = eye(size(G));
G_tilde_mp = W2*G*W1;
% figure
% bode(G_tilde_mp)
%% Exercise: 3.1.2
% Designing diagonal PI controllers
PM_d = pi/3;
Wc_d = 0.1;
% F11
gt11 = G_tilde_mp(1,1);
[Kp1,Ki1] = PIcontrol(PM_d,Wc_d,gt11);
F11 = Kp1*(1+(Ki1/s));
% F22
gt22 = G_tilde_mp(2,2);
[Kp2,Ki2] = PIcontrol(PM_d,Wc_d,gt22);
F22 = Kp2*(1+(Ki2/s));
% F_tilde
F_tilde = [F11,0;0,F22];
% F(s)
F_mp = W1*F_tilde;
%% Exercise: 3.1.3
S_mp = minreal(1/(eye(2,2)+(G*F_mp)));
T_mp = minreal(S_mp*G*F_mp);
[sv_S,w_S] = sigma(S_mp);
[sv_T,w_T] = sigma(T_mp);
[sv_S1_max,id_S1] = max(sv_S(1,:));
[sv_S2_max,id_S2] = max(sv_S(2,:));
[sv_T1_max,id_T1] = max(sv_T(1,:));
[sv_T2_max,id_T2] = max(sv_T(2,:));

%% DYNAMIC DECOUPLING
%% Exercise: 3.2.1
RGA = G0_mp .* inv(G0_mp)';
W2d = eye(2,2);
g11 = G(1,1);
g12 = G(1,2);
g21 = G(2,1);
g22 = G(2,2);
W1d = minreal([1,-g12/g11; -g21/g22,1]);
G_tilde_d = minreal(W2d*G*W1d);
figure
bode(G_tilde_d)
hold on 
grid on
hold off
%% Exercise: 3.2.2
% Designing diagonal PI controllers
% F11
gtd11 = G_tilde_d(1,1);
[Kpd1,Kid1] = PIcontrol(PM_d,Wc_d,gtd11);
Fd11 = Kpd1*(1+(Kid1/s));
% F22
gtd22 = G_tilde_d(2,2);
[Kpd2,Kid2] = PIcontrol(PM_d,Wc_d,gtd22);
Fd22 = Kpd2*(1+(Kid2/s));
% F_tilde
Fd_tilde = [Fd11,0;0,Fd22];
% Controller F(s)
F = W1d*Fd_tilde;
% %% Exercise: 3.2.3
% Sd_mp = minreal(1/(eye(2,2)+(G*F_old)));
% Td_mp = minreal(Sd_mp*G*F_old);
% [svd_S,w_S] = sigma(Sd_mp);
% [svd_T,w_T] = sigma(Td_mp);
% [svd_S1_max,id_S1] = max(svd_S(1,:));
% [svd_S2_max,id_S2] = max(svd_S(2,:));
% [svd_T1_max,id_T1] = max(svd_T(1,:));
% [svd_T2_max,id_T2] = max(svd_T(2,:));
% %% Glover-McFarlane Robust Loop Shaping
% alpha = 1.1;
% L = minreal(G*F_old);
% [Fr,gam] = rloop(L,alpha);
% F = minreal(F_old * Fr);
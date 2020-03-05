%% Decentralized Control for Non- Minimum Phase
%% From the RGA, we observe that we should pair U1 -> Y2 and U2 -> Y1
clear;close all; clc;
%% Import Min Phase MIMO System
sys = nonminphase;
%% Extract indi. transfer functions
[num,den] = tfdata(sys);
G_11 = tf(num{1,1},den{1,1});
G_12 = tf(num{1,2},den{1,2});
G_21 = tf(num{2,1},den{2,1});
G_22 = tf(num{2,2},den{2,2});
% System Transfer Matrix
G = [G_11 G_12; G_21 G_22];
%% System Requirements
w_c = 0.02;
phi_m = pi/3;
%% Extract the phase of G_11 and G_22
[~,phi_12] = bode(G_12,w_c);
[~,phi_21] = bode(G_21,w_c);
%% Calculating Ti from [phi_ij +atan(W_c*Ti)-pi/2-phi_m = -pi]
Ti1 = (1/w_c) * tan(phi_m - pi/2 - (phi_21*pi/180));
Ti2 = (1/w_c) * tan(phi_m - pi/2 - (phi_12*pi/180));
%% Loop Gain L = FG
s = tf('s');
L_11 = G_12 * (1 + (1/(s*Ti2)));
L_22 = G_21 * (1 + (1/(s*Ti1)));
%% Mag from Bode
[k2,~] = bode(L_11,w_c);
[k1,~] = bode(L_22,w_c);
K1 = 1/k1;
K2 = 1/k2;
%% Decentralized controller
f12 = K1*(1 + (1/(s*Ti1)));
f21 = K2*(1 + (1/(s*Ti2)));
f11 = 0;
f22 = 0;
F = [f11 f12; f21 f22];
L = G*F;
%% Validating the performance
figure
margin(L(1,1))
figure
margin(L(2,2))
figure
bode(L)
%% Task 3.2.2
%% Sensitivity Function
S = minreal(inv(eye(2)+L));
%% Complimentary Sensitivity Function 
T = minreal(inv(eye(2)+L) * L);
figure
sigma(S)
hold on
sigma(T)
title('Singular Value Plot for S and T - NMP')
legend('S','T')
hold off
%% Task 3.2.3
sim('closedloop');

figure
plot(uout)
hold on
title('Plot of Control Outputs u1 and u2 - NMP')
legend('u1','u2')
grid on
hold off

figure
plot(yout)
hold on

title('Plot of Outputs y1 and y2 - NMP')
legend('y1','y2')
grid on
hold off

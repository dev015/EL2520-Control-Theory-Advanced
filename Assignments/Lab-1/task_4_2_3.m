%% 2 DOF Controller Design
clear; close all; clc;
G = tf([8000],[1,21,420,400]);
Gd = tf([10],[1,1]);
[~,~,~,w_c] = margin(Gd);
%w_i = 0.55*w_c;
w_i = 0.57*w_c;
%% Proper Controller
p = 10*w_c;
Fy_improper = tf([1,w_i],[1,0]) * G^-1 * Gd;
Fy_proper = Fy_improper * tf([p^2],[1,2*p,p^2]);
%% Lead Compensation
w_d = w_c + 2;
[mag_uc,~,~] = bode(Fy_proper*G,w_d);
L1 = Fy_proper*G;
[Gm_uc,Pm_uc,Wcg_uc,Wcp_uc] = margin(L1);
Pm_d = 30;
Sf = 32;
Phi_max = abs(Pm_d + Sf - Pm_uc);
beta = (1 - sind(Phi_max))./(1 + sind(Phi_max));
tau_d = 1/(w_d * (sqrt(beta)));
K = sqrt(beta)/mag_uc;
% Lead Compensator TF
num_lead = [K*tau_d, K];
den_lead = [beta*tau_d, 1];
F_lead = tf(num_lead,den_lead);
Fy = minreal(Fy_proper * F_lead);
% Loop Gain
L = Fy*G;
%% Sensitivity Function
S = 1/(1+L);
%% Complimentary Sensitivity Function
T = 1-S;
%% CLTF from d to y
Gcd = minreal(Gd*S);
%% Selecting Tau in order to attain the specifications wrt r
% Tau = [0.09 0.089 0.088];
% for i = 1: length(Tau)
%     tau = Tau(i);
%     Fr = tf([1],[tau,1]);
%     Gcl = minreal(Fr*L/(1+L));
%     figure
%     step(Gcl)
% end
%tau = 0.088;
tau = 0.1;
Fr = tf([1],[tau,1]);
Gcl = minreal(Fr*L/(1+L));
%% Size of Control Signal
ur = minreal(Fy*Fr*S);
ud = minreal(Fy*Gd*S);
%% Plotting
figure
bode(S)
hold on
bode(T)
legend('S','T')
hold off

figure
step(Gcd)

figure
step(Gcl)

figure
step(ur)
hold on
step(ud)
legend('U-Ref','U-Dist')

max(step(ur)) - min(step(ud))
max(step(ud)) - min(step(ur))
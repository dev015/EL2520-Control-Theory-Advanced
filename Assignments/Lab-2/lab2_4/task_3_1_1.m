%% This script is the solution to the Part 1 of Lab2
clear;close; clc;
%% Import Min Phase MIMO System
% sys = minphase;
%% Import Non-minimum Phase MIMO system
sys = nonminphase;
%% Extract indi. transfer functions
[num,den] = tfdata(sys);
G_11 = tf(num{1,1},den{1,1});
G_12 = tf(num{1,2},den{1,2});
G_21 = tf(num{2,1},den{2,1});
G_22 = tf(num{2,2},den{2,2});
% System Transfer Matrix
G = [G_11 G_12; G_21 G_22];
%% Calculating the poles
p11 = pole(G_11);
p12 = pole(G_12);
p21 = pole(G_21);
p22 = pole(G_22);
%% Calculating the zeros
z11 = tzero(G_11);
z12 = tzero(G_12);
z21 = tzero(G_21);
z22 = tzero(G_22);
%% Task 3.1.2
p = eig(sys.A);
z = tzero(G_11*G_22 - G_12*G_21);
%% Task 3.1.3
figure
sigma(G)
[sv,w] = sigma(G);
[sv1_max,idx_1] = max(sv(1,:));
[sv2_max,idx_2] = max(sv(2,:));
[sv_max,id] = max([sv1_max, sv2_max]);
if id ==1
    idx = idx_1;
else
    idx = idx_2;
end
w_max = w(idx);
%% Task 3.1.4
RGA = evalfr(G,0) .* evalfr(inv(G)',0)
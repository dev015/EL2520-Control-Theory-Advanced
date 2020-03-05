%% Disturbance Attenuation
clear; close all; clc;
G = tf([8000],[1,21,420,400]);
Gd = tf([10],[1,1]);
[~,~,~,Wc] = margin(Gd);
G1 = tf([Wc],[1,0]);
%% Improper Controller
Fy_improper = series(G1,G^-1);
% Loop Gain
L_improper = G*Fy_improper;
% CLTF
Gdc_improper = Gd / (1+L_improper);
%% Proper Controller 
% Add 2 poles in order to make the Fy proper and ensure that the Bode plot
% of closed loop tf and step response remains the same
%% Observation: Take both poles at least at 70*Wc
p = 70*Wc;
Fy_proper = Fy_improper * tf([p^2],[1,2*p,p^2]);
% Loop Gain
L_proper = G*Fy_proper;
% CLTF
Gdc_proper = Gd / (1+L_proper);
%% Plotting
figure(1)
margin(L_improper)
hold on 
margin(L_proper)
hold off

figure(2)
margin(Gdc_improper)
hold on 
margin(Gdc_proper)
hold off

figure(3)
stepplot(Gdc_improper)
hold on 
stepplot(Gdc_proper)
hold off
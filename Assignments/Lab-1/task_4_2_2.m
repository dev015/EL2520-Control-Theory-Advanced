%% Disturbance Attenuation
clear; close all; clc;
G = tf([8000],[1,21,420,400]);
Gd = tf([10],[1,1]);
[~,~,~,Wc] = margin(Gd);
%% Reconstruct Fy with different Wi values
% Wi = [5*Wc, 4*Wc, 3*Wc, 2*Wc];
% for i = 1:length(Wi)
%     w = Wi(i);
%     Fy = tf([1,w],[1,0]) * G^-1 * Gd;
%     Gdc = Gd/(1+(Fy*G));
%     figure
%     step(Gdc)
%     hold on 
%     grid on
%     hold off
% end
Wi = 0.57*Wc;
%Wi = 5.5*Wc;
%% Improper Controller
Fy_improper = tf([1,Wi],[1,0]) * G^-1 * Gd;
Gdc_improper = Gd/(1+(Fy_improper*G));
%% Proper Controller
p = 10*Wc;
Fy_proper = Fy_improper * tf([p^2],[1,2*p,p^2]);
Gdc_proper = Gd/(1+(Fy_proper*G));
step(Gdc_improper)
hold on
step(Gdc_proper)
legend('Improper','Proper')
grid on
hold off
% Gc_proper = feedback(Fy_proper*G,1);
% figure
% stepplot(Gc_proper)
%% This function computes the PI controller
function [Kp,Ki] = PIcontrol(PM_d,Wc_d,G)
s = tf('s');
[~,ph_uc_deg] = bode(G,Wc_d);
ph_uc_rad = ph_uc_deg*pi/180;
Ki = Wc_d/tan(-pi/2+PM_d-ph_uc_rad);
F = 1+Ki/s;
L = G*F;
[mag,~] = bode(L,Wc_d);
Kp = 1/mag;
end
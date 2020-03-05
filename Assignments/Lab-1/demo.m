clear
close all

s = tf('s');

%% Process
G = 20 / ((s+1)*((s/20)^2 + s/20 + 1));

%% Disturbance
Gd = 10 / (s+1);

% The disturbance's crossover frequency
[~,~,~,wc] = margin(Gd)


%% Fy

% Safe zone: wd > wc
wd = wc + 5;

wi = 0.5 * wc;

p_1 = 5 * wc;
p_2 = 5 * wc;


Fy = (s + wi) * Gd / (s * G);
Fy = Fy * p_1 * p_2 / ((s+p_1)*(s+p_2));

[mag, ph] = bode(Fy * G, wd)

% The desired phase margin
ph_desired = 30;

ph = ph + 180
phi = abs(ph_desired - ph);

b = (1 - sin(phi * pi / 180)) / (1 + sin(phi * pi / 180))
Td = 1 / (wd * sqrt(b))
K = sqrt(b) / mag


Fy = Fy * K * (Td *s + 1) / (b * Td * s + 1);
Fy = minreal(Fy);

%% L
L = Fy * G;

%% Fr
t = 0.1;
Fr = 1 / (1 + t*s);

%% Sensitivity tranfer function

S = 1 / (1 + L);

%% Complementary sensitivity transfer function

T = 1 - S;

%% The closed-loop transfer function from d to y

Cd = minreal(Gd * S);

%% The closed-loop transfer function from r to y

Cr = minreal(Fy * Fr * G * S);

%% Control input with regard to the reference

ur = minreal(Fy * Fr * S);

%% Control input with regard to the disturbance

ud = minreal(Fy * Gd * S);

%% Plot figures
bode(S);
hold on
bode(T)

figure
step(Cr)
stepinfo(Cr)

figure
step(Cd)
stepinfo(Cd)

figure
step(ur)
hold on
step(ud)


max(step(ur)) - min(step(ud))
max(step(ud)) - min(step(ur))
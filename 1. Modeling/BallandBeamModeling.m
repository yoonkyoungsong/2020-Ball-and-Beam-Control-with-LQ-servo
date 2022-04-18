%% Ball and Beam system modeling
clear all; clc;

% batch폼으로 구한 모터 TF (wm=36.1801)
s           = tf('s');
wm          = sqrt(1309);
num2        = [5.94e04];
den2        = [1 47.56 1309];

Gm_2        = tf(num2, den2);

%1차로 근사한 모터TF (V2V)
Km1          = 5.94e04/wm^2;
% taum        = 1/wm;
taum        = 0.036331449898502;
num1        = [Km1];
den1        = [taum 1];

Gm_1        = tf(num1, den1);
% step(Gm_2order); hold on; step(Gm_1order);

% [A B C D]=tf2ss(num2,den2);

g           = 9.81;                 % grav acceleration [m/s^2]

%beam
lbeam       = 0.42;                 % length of beam    [m]
mbeam       = 0.2236;               % weight of beam    [kg]
Jbeam       = 1/12*mbeam*lbeam^2;    % moment of inertia

%ball
wball       = 0.11015;              % weight of ball    [kg]
mball       = wball/g;              % mass of ball      [kg]

%motor
mgear       = mball;
%Rm          = 19;
% J           = taum/Rm;
J           = Jbeam;
Rm          = taum/J;
bm          = 0;
Km          = sqrt(wm*Rm*J);
Ke          = Km;

a1          = -(Km*Ke)/(J*Rm);
a2          = -(wball)/J;
a3          = (5*g)/7;
b1          = Km/(J*Rm);

A           = [0   1   0   0;
               0   a1  a2  0;
               0   0   0   1;
               a3  0   0   0];
B           = [0;  b1; 0;  0];
C           = [0   0   1   0];
D           = zeros(1,1);

sys         = ss(A, B, C, D);

TF          = tf(sys);

% figure (1)
% step(TF); hold off;
% 
% figure (2)
% pzmap(sys); hold off;

wn          = 7;
zeta        = 0.9;

% controllability check
Co          = ctrb(A,B);
unco        = length(A) - rank(Co);

% open loop
open_poles  = eig(A);
sysGo       = ss(A, B, C, D);
Go          = tf(sysGo);

% pole-placement
dominant1   = - zeta*wn + j*wn*sqrt(1-zeta^2);
dominant2   = - zeta*wn - j*wn*sqrt(1-zeta^2);
indominant1 = real(dominant1)*2;
indominant2 = real(dominant2)*2;
% indominant1 = real(dominant1)*4 + j*imag(dominant1)*5;
% indominant2 = real(dominant2)*4 + j*imag(dominant2)*5;
poles = [dominant1 dominant2 indominant1 indominant2]

close all;
%closed loop
K = acker(A,B,poles)
%K = [1 1 1 1];


Acl = A-B*K;
Bcl = B;
Ccl = C;
Dcl = D;

sysGcl = ss(Acl, Bcl, Ccl, Dcl);
Gcl = tf(sysGcl);

% figure, pzmap(Go,Gcl), legend("Go","Gcl")
% figure, bodemag(Gcl), legend("Gcl")
% hold on, subplot(1,2,2), step(Gcl), legend("Gcl")



%% open loop model : nyquist plot => margin check
close all; clc;

initTheta=0;%30/180*pi;
initX = 0.07;%0.03;

sys     = linmod('OpenLoopBallandBeam');

[Ns,Ds] = ss2tf(sys.a, sys.b, sys.c, sys.d);
Go      = -tf(Ns,Ds); 
Go      = minreal(Go);

figure, nyquist(Go)
xlim([-1 1]), ylim([-1 1])

%% closed loop linear-nonlinear model 
close all; clc;

initTheta=30/180*pi;
initX = 0.00;%0.03;

out = sim('ClosedLoopBallandBeam.slx');

time = out.FSF_nonlinear.time;

nontheta =out.FSF_nonlinear.signals.values(:,1);
nontheta_dot = out.FSF_nonlinear.signals.values(:,2);
nonx = out.FSF_nonlinear.signals.values(:,3);
nonx_dot =out.FSF_nonlinear.signals.values(:,4);

theta = out.FSF_linear.signals.values(:,1);
theta_dot = out.FSF_linear.signals.values(:,2);
x = out.FSF_linear.signals.values(:,3);
x_dot = out.FSF_linear.signals.values(:,4);

figure()
subplot(2,1,1)
plot(time,theta/pi*180,'y','LineWidth',2); grid on; box on;
hold on, plot(time,nontheta/pi*180,'k--','LineWidth',2); grid on; box on;
legend('linear','nonlinear')

xlabel('t [sec]'); ylabel(' \theta [deg]')
title('Nonlinear Ball and Beam Model Output');
subplot(2,1,2)
plot(time,x*100,'y','LineWidth',2); grid on; box on;
hold on, plot(time,nonx*100,'k--','LineWidth',2); grid on; box on;
legend('linear','nonlinear')
xlabel('t [sec]'); ylabel('y [cm]')
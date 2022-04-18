clc; clear all; close all; 

% Simulation Time
Ts = 0.005;
T_start = 0.0;
T_final = 80.0;


%% Model Parameters%% motor transfer function and reduction
clear all; close all; clc;
s           = tf('s');
wm          = sqrt(899.8);
num2        = [48390*pi/180];
den2        = [1 58.28 899.8];
Gm_2        = tf(num2, den2);
% Km2         = (num2(1)/180)*pi;

%1st TF
Km1         = num2(1)/wm^2;
taum        = 1/wm; % taum        = 1/wm;
num1        = [Km1];
den1        = [taum 1];

Gm_1        = tf(num1, den1);
% step(Gm_2order); hold on; step(Gm_1order);

% [A B C D]=tf2ss(num2,den2);

% full-state feedback
%from ball and beam dynamics
% close all; clear all; clc;

g           = 9.81;                     % grav acceleration [m/s^2]

%ball
wball       = 0.11015;                  % weight of ball    [kg]
mball       = wball/g;                  % weight of ball    [kg]
Rball       = 0.015;

%beam
lbeam       = 0.42;                     % length of beam    [m]
wbeam       = 0.2236;                   % weight of beam    [kg]
mbeam       = wbeam;%/g;                  % weight of beam    [kg]
Jbeam       = 1/12*(mbeam)*lbeam^2;     % moment of inertia 
Jball       = 2/5*(mball)*(Rball)^2;
Jcont       = mball*(lbeam+Rball)^2;


%motor
bm          = 0;
J           = Jbeam+0.00000101055045871557+0.002124682339449486;
Ke          = 1/Km1;
Rm          = (taum*Ke*Ke)/J;

a1          = -(Ke*Ke)/(J*Rm);
a2          = -(mball*g)/J;
a3          = (5*g)/7;
b1          = Ke/(J*Rm);

A           = [0   1   0   0;
               0   a1  a2  0;
               0   0   0   1;
               a3  0   0   0];
B           = [0;  b1; 0;  0];
C           = [0   0   1   0];
D           = 0;

sys         = ss(A, B, C, D);

TF          = tf(sys);
 
N = 0;

% w1=0.2;
% w2=0.0;
% w3=0.3;
% w4=0.0;
% w5=0.5;
% R_servo = 0.05;

w1=0.3;
w2=0.0;
w3=0.2;
w4=0.1;
w5=0.4;
R_servo = 0.05;

A_servo = [A zeros(4,1); 0 0 -1 0 zeros(1,1)];
B_servo = [B; zeros(1,1)];
Q_servo = diag([w1/(deg2rad(20))^2 w2/(deg2rad(100))^2 w3/(0.024)^2 w4/(0.1)^2 w5/(0.02)^2]);
%Q_servo = diag([w1/(deg2rad(80))^2 w2/(deg2rad(200))^2 w3/(0.024)^2 w4/(0.4)^2 w5/(0.02)^2]);
%Q_servo = diag([w1 w2 w3 w4 w5])
[ K_servo, S_servo, P_servo ] = lqr(A_servo, B_servo, Q_servo, R_servo, N);
disp(K_servo)

initTheta=30/180*pi;
initX = 0.00;%0.03;

wantx1=5;
wantx2=10;

LQservo = sim('LQservo_ClosedLoopBallandBeam.slx');

time = LQservo.tout;

data1=load("LQservo_state2.00.txt");
data2=load("LQservo_theta_omega2.00.txt");
data3=load("LQservo_Vcmd_x2.00.txt");

ref=LQservo.ref;

theta = LQservo.LQ_state(:,1);
theta_dot = LQservo.LQ_state(:,2);
x = LQservo.LQ_state(:,3);
x_dot =LQservo.LQ_state(:,4);

% figure()
% plot(time,theta/pi*180,'y--','LineWidth',2); grid on; box on;
% hold on, grid on,
% legend('actual','simaulation')
% xlabel('t [sec]'); ylabel(' \theta [deg]')
% title('Nonlinear Ball and Beam Model Output');

figure()
plot(time,x,'b','LineWidth',2);
hold on, grid on,
plot(time,ref,'r--','LineWidth',2);
plot(data3(:,1),data3(:,3)*100,'g','LineWidth',2)
legend("simulation","referenceX","actual")
xlabel('time [sec]'); ylabel('y [cm]')


% figure, pzmap(Go,Gcl), legend("Go","Gcl")
% figure, bodemag(Gcl), legend("Gcl")
% hold on, subplot(1,2,2), step(Gcl), legend("Gcl")


%% open loop model : nyquist plot => margin check
close all; clc;

sys     = linmod('OpenLoopBallandBeam');

[Ns,Ds] = ss2tf(sys.a, sys.b, sys.c, sys.d);
Go      = -tf(Ns,Ds); 
Go      = minreal(Go);

% figure, nyquist(Go)
% xlim([-1 1]), ylim([-1 1])

figure, margin(Go)


%% LQservo simulation
close all; clc;

initTheta=30/180*pi;
initX = 0.00;%0.03;

wantx1=5;
wantx2=10;

LQservo = sim('LQservo_ClosedLoopBallandBeam.slx');

time = LQservo.tout;

data1=load("LQservo_state0.00.txt");
data2=load("LQservo_theta_omega0.00.txt");
data3=load("LQservo_Vcmd_x0.00.txt");


theta = LQservo.LQ_state(:,1);
theta_dot = LQservo.LQ_state(:,2);
x = LQservo.LQ_state(:,3);
x_dot =LQservo.LQ_state(:,4);

% figure()
% plot(time,theta/pi*180,'y--','LineWidth',2); grid on; box on;
% hold on, grid on,
% legend('actual','simaulation')
% xlabel('t [sec]'); ylabel(' \theta [deg]')
% title('Nonlinear Ball and Beam Model Output');

figure()
plot(data3(:,1),data3(:,3)*100,'b','LineWidth',2)
hold on, grid on,
plot(time,x,'y--','LineWidth',2); grid on; box on;
legend('actual','simaulation')
xlabel('t [sec]'); ylabel('y [cm]')

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
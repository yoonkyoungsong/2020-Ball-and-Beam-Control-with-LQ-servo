%% motor transfer function and reduction
clear all; clc
% batch TF (wm=36.1801)
s           = tf('s');
wm          = sqrt(899.8);
num2        = [4.839e04];
den2        = [1 58.28 899.8];
Gm_2        = tf(num2, den2);
Km2         = num2(1);

V2W         = Km2/den2(3);

%1st TF (V2V)
Km1         = 4.839e04/wm^2;%/V2W
taum        = 0.027639499641139; % taum        = 1/wm;
num1        = [Km1];
den1        = [taum 1];

Gm_1        = tf(num1, den1);
% step(Gm_2order); hold on; step(Gm_1order);

% [A B C D]=tf2ss(num2,den2);


%%  OS와 Tr로 zeta, Wc 범위 설정

clear all; close all; clc;

%controller specification
%OS=0.17; % %값
%Tr=0.07;
OS=0.2;
Tr=0.07;

%Rc
zetac=sqrt((log(OS))^2/(log(OS)^2+pi^2)); % >
%wc=pi/Tr*sqrt(1-zetac^2);
wc = (1-0.4267*zetac+2.917*zetac^2)/Tr;


%% zeta, Wc로 kp,kd,ki 값
clear all; close all; clc;

%innerloop
zetacin= 0.7;
wcin=38;

% batch폼으로 구한 모터 TF (wm=36.1801)
s           = tf('s');
wm          = sqrt(899.8);
num2        = [4.839e04/180*pi];
den2        = [1 58.28 899.8];

Gm_2        = tf(num2, den2);

%1차로 근사한 모터TF (V2V)
Km1          = 4.839e04/wm^2/180*pi;
taum        = 1/wm;
%taum        = 0.027639499641139;
num1        = [Km1];
den1        = [taum 1];


%wm=sqrt(den(3));
Km=num2/wm^2;
Tm=1/wm;

Kdin=(2*zetacin*wcin*Tm-1)/Km;
Kpin=wcin^2*Tm/(Kdin*Km);


%outerloop
zetacout= 0.7;
wcout=4.5;

g=9.8;
a3 = 5*g/7;

Kdout=(2*zetacout*wcout)/a3;
Kpout=wcout^2/a3;
Kiout=0;


%% check margine => nyquist plot
close all; clc;

initTheta=30/180*pi;
initThetaout=0;
initXout=0.03;


sysin     = linmod('OpenLoopAngular');

[Nsin,Dsin] = ss2tf(sysin.a, sysin.b, sysin.c, sysin.d);
Goin      = -tf(Nsin,Dsin); 
Goin      = minreal(Goin);

figure, nyquist(Goin)
xlim([-1 1]), ylim([-1 1])


sysout     = linmod('OpenLoopPosition');

[Nsout,Dsout] = ss2tf(sysout.a, sysout.b, sysout.c, sysout.d);
Goout      = -tf(Nsout,Dsout); 
Goout      = minreal(Goout);

Go=a3*tf([Kdout, Kpout],[1 0 0]);

figure, nyquist(Goout)
xlim([-1 1]), ylim([-1 1])

figure, nyquist(Go)
xlim([-1 1]), ylim([-1 1])

%%

figure, pzmap(Goout)
hold on
pzmap(Gclout)


%% simulation

close all; clc;

wanttheta1=10;
wanttheta2=30;

wantx1=5;
wantx2=10;

% data1=load("inner_Vcmd_Gyro0.00.txt");
% data2=load("inner_deg_pos0.00.txt");

data1=load("refpulse_Vcmd_Gyro0.00.txt");
data2=load("refpulse_deg_pos0.00.txt");

angular = sim("ClosedLoopAngular.slx");
position = sim("ClosedLoopPosition.slx");

% figure()
% plot(angular.tout,angular.theta,'b','linewidth',2);
% hold on,
% plot(angular.tout,angular.wantedtheta,'y--','linewidth',2);
% grid on
% legend("simulation","target point")
% xlim([0 45])
% title("Angular controller simulation")

figure()
plot(position.tout-20,position.x,'b','linewidth',2);
hold on,
plot(position.tout-20,position.wantedx,'r--','linewidth',2);
plot(data2(:,1),data2(:,3)*100,'g','linewidth',2);
grid on
legend("simulation","referenceX","actual")
xlim([0 25])
title("Classical Controll G_{classcial}= x_c / x_{ref} [-]")
xlabel("time[sec]"),ylabel("positionX[cm]")

%% innerloop step response

wantx1=5;
wantx2=9;

data1=load("innerstep_Vcmd_Gyro0.00.txt");
data2=load("innerstep_deg_pos0.00.txt");

angular = sim("step_ClosedLoopAngular.slx");

figure()
plot(angular.tout,angular.theta*180/pi,'b','linewidth',2);
hold on,
plot(angular.tout,angular.wantedtheta*180/pi,'r--','linewidth',2);
plot(data2(:,1),data2(:,2)*180/pi,'g','linewidth',2);
grid on
legend("simulation","referenceX","actual")
xlim([0 0.5])
title("Angular controller step response")
xlabel("time[sec]"),ylabel("gyro[deg]"),


%% outerloop step response

wantx1=5;
wantx2=9;

data1=load("classic_Vcmd_Gyro0.00.txt");
data2=load("classic_deg_pos0.00.txt");

position = sim("step_ClosedLoopPosition.slx");
figure()
plot(position.tout,position.x,'b','linewidth',2);
hold on,
plot(position.tout,position.wantedx,'r--','linewidth',2);
plot(data2(:,1)-0.12,data2(:,3)*100,'g','linewidth',2);
grid on
legend("simulation","referenceX","actual")
xlim([0 3])
title("Position controller step response")
xlabel("time[sec]"),ylabel("postion[cm]"),

%% Motor static characteristic
clc; clear all; close all;

addpath('CW_data');
addpath('CCW_data');

n=22;

CCWdata=zeros(200,n);
CWdata=zeros(200,n);

% CCW(i)=zeros(1,n);
% CW(i)=zeros(1,n);

for i=1:1:n
    
    data1 =load(sprintf("CWdata_%.2f.txt",(i+12)/10));
    data2 = load(sprintf("CCWdata_%.2f.txt",(i+12)/10));
    
    CCWdata(:,i)=data1(:,3);
    CWdata(:,i)=data2(:,3);
    
    CCWVcmd(i)=mean(data1(20:200,2));
    CWVcmd(i)=mean(data2(20:200,2));
    
    CCW(n+1-i)=mean(CCWdata(20:200,i));
    CW(i)=mean(CWdata(20:200,i));
end

x=-3.4:0.1:3.4;
x=-x;
linear=[CCW zeros(1,25) CW];

plot(x,linear,'b','linewidth',2); grid on
xlim([-5 5]); hold on;
title("Motor Linearization")
xlabel("Vcmd[V]"),ylabel("GyroW[deg/s]")



% %% Linearization motor static characteristic
% clc; clear all; close all;

addpath('LCW_data2');
addpath('LCCW_data2');

n=51;

CCWdata=zeros(200,n);
CWdata=zeros(200,n);

for i=1:1:51  
    
    data1 =load(sprintf("LCWdata_%.2f.txt",(i-1)/10));
    data2 = load(sprintf("LCCWdata_%.2f.txt",(i-1)/10));
    
    CCWdata(:,i)=data1(:,3);
    CWdata(:,i)=data2(:,3);
    
    CCWVcmd(i)=mean(data1(20:200,2));
    CWVcmd(i)=mean(data2(20:200,2));
    
    CCW(i)=mean(CCWdata(20:200,i));
    CW(i)=mean(CWdata(20:200,i));
    
end

Vcmd=[CCWVcmd CWVcmd];
linear=[CCW CW];

plot(Vcmd,40*Vcmd,'r','linewidth',2);
xlim([-5 5]); ylim([-250 250]); hold on;
plot(Vcmd,linear,'kx','linewidth',2); grid on
legend("row","ideal","actual")

%% curvfitting for static motor characterastic
close all; clc

%CW
cwVcmdDead=0.5;
cwVssDead =1.4;

cwVcmdSatuate=3.5084;
cwVssSatuate=2.27;

cwVcmdMid = 1.625;
cwVssMid = 1.684;


% CCW
ccwVcmdDead=-0.585;
ccwVssDead=-1.4;

ccwVcmdSatuate=-3.5;
ccwVssSatuate=-2.2609;

ccwVcmdMid = -1.625;
ccwVssMid = -1.6534;



% cwVcmdDead=0.28;
% cwVssDead =1.405;
% cwVcmdMid =1.02;
% cwVssMid = 1.875;
% cwVcmdSatuate=1.875;
% cwVssSatuate=2.51;
% 
% ccwVcmdDead=-0.305;
% ccwVssDead=-1.405;
% ccwVcmdMid = -1.02;
% ccwVssMid = -1.845;
% ccwVcmdSatuate=-1.875;
% ccwVssSatuate=-2.48;
    
    
% y=ax^2+bx+c
cwX=[ cwVcmdDead^2, cwVcmdDead, 1 ;
         cwVcmdMid^2, cwVcmdMid, 1 ; 
         cwVcmdSatuate^2, cwVcmdSatuate, 1 ; ];
cwY=[ cwVssDead; cwVssMid; cwVssSatuate] ;
cwCoeff = cwX\cwY;

%  y=ax^2+bx+c
ccwX=[ ccwVcmdDead^2, ccwVcmdDead, 1 ;
         ccwVcmdMid^2, ccwVcmdMid, 1 ; 
         ccwVcmdSatuate^2, ccwVcmdSatuate, 1 ; ];
ccwY=[ ccwVssDead; ccwVssMid; ccwVssSatuate] ;
ccwCoeff = ccwX\ccwY;



Vcmd=-5:0.005:5;
rowWout = MotorStaticV2W(Vcmd);

Vss = Linearization(Vcmd, cwCoeff, ccwCoeff);
Wout = MotorStaticV2W(Vss);

figure()
plot(x,linear); grid on
hold on
plot(Vcmd, rowWout)%'LineWidth',2);
plot(Vcmd, Wout)%,'LineWidth',2);
grid on
plot(Vcmd, 40*Vcmd)%,'LineWidth',2);
title("Linearization of Motor")
xlabel("Vcmd[V]")
ylabel("Wout[deg/s]")
legend("rowMotor","actual Linearization","ideal Linearization")

ylim([-230 230])

% figure()
% plot(Vcmd, Vcmd)
% hold on
% plot(Vcmd,Vss)


%%

figure()
data = load("CCWdata_2.00.txt");
plot(data(:,1),data(:,3)), hold on,
plot(data(:,1),40*data(:,2))


%% curvfitting for static motor characterastic
close all; clc

%CW
cwVcmdDead=0.75;
cwVssDead =1.4;

cwVcmdSatuate=4.689;
cwVssSatuate=2.4;

cwVcmdMid = 2.47;
cwVssMid = 1.8;


% CCW
ccwVcmdDead=-0.815;
ccwVssDead=-1.4;

ccwVcmdSatuate=-4.79;
ccwVssSatuate=-2.4;

ccwVcmdMid = -2.46;
ccwVssMid = -1.8;
    
    
% y=ax^2+bx+c
cwX=[ cwVcmdDead^2, cwVcmdDead, 1 ;
         cwVcmdMid^2, cwVcmdMid, 1 ;
         cwVcmdSatuate^2, cwVcmdSatuate, 1 ; ];
cwY=[ cwVssDead; cwVssMid; cwVssSatuate] ;
cwCoeff = cwX\cwY;

%  y=ax^2+bx+c
ccwX=[ ccwVcmdDead^2, ccwVcmdDead, 1 ;
         ccwVcmdMid^2, ccwVcmdMid, 1 ; 
         ccwVcmdSatuate^2, ccwVcmdSatuate, 1 ; ];
ccwY=[ ccwVssDead; ccwVssMid; ccwVssSatuate] ;
ccwCoeff = ccwX\ccwY;



Vcmd=-5:0.005:5;
rowWout = MotorStaticV2W(Vcmd);

Vss = Linearization(Vcmd, cwCoeff, ccwCoeff);
Wout = MotorStaticV2W(Vss);

figure()
plot(Vcmd, rowWout)
hold on
plot(Vcmd, Wout)
grid on
plot(Vcmd, 30*Vcmd)
title("Linearization of Motor")
xlabel("Vcmd[V]")
ylabel("Wout[deg/s]")
legend("rowMotor","actual Linearization","ideal Linearization")

ylim([-200 200])



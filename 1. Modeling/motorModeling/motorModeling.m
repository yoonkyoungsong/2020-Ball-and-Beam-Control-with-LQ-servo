%% Calculate Phase Shift
clear all; close all; clc;

addpath('freqcheck');

Ts=0.005;
%t=0:ts:1-ts;
%f = 6.4; 
Wc=60;

n=200;

for i=5 : 0.1 : 7 %
    
    f = i; 

    %rowdata =load("data_6.60.txt");
    dataName = sprintf("sindata_freq%.2f.txt", i);
    str=sprintf(dataName);
    rowdata = load(str);

    time=rowdata(1:n,1);
    Vcmd= rowdata(:,2);
    %Vss= Linearization(Vcmd);
    Wout = rowdata(:,3);

    %Vout = lowpass(Vout,Wc,ts);


    data = zeros(length(Vcmd),2);

    data(:,1)=Vcmd;
    data(:,2)=Wout;

    UNIT_RAD2DEG = 180/pi ;
    UNIT_DEG2RAD = pi/180 ;

    %f = 6.4; % input signal frequency
    time = time; % time dimension matching 

    meas1 = data(1: n,1) ;
    meas2 = data(1: n,2) ;

    % 입출력 신호 크기 및 위상에 대한 최소자승 추정치 (least squares estimates)

    coef1 = lsqcurvefit(@(coef1, time) coef1(1)*sin( 2*pi*f*time + coef1(2) ) + coef1(3), [max(meas1) 0 (max(meas1)+min(meas1))/2], time, meas1) ;
    coef2 = lsqcurvefit(@(coef2, time) coef2(1)*sin( 2*pi*f*time + coef2(2) ) + coef2(3), [max(meas2) 0 (max(meas2)+min(meas2))/2], time, meas2) ;

    eMag1  = coef1(1) ; % estimated magnitude
    ePhs1  = coef1(2) ; % estimated phase
    eBias1  = coef1(3) ; % estimated bias

    eMag2  = coef2(1) ; % estimated magnitude
    ePhs2  = coef2(2) ; % estimated phase
    eBias2  = coef2(3) ; % estimated bias


    ey1 = eMag1 * sin( 2*pi*f*time + ePhs1 ) + eBias1 ; % estimated y
    ey2 = eMag2 * sin( 2*pi*f*time + ePhs2 ) + eBias2 ; % estimated y

    % 결과 그래프

    figure,  plot(time, meas1, 'b', time, ey1, 'c:', 'linewidth', 2), grid on ;
    hold on, plot(time, meas2, 'g', time, ey2, 'm:', 'linewidth', 2), grid on ;
    axis([0 1 -250 250])
    xlabel('time [sec]'), ylabel('measured and estimated [deg/s]'), legend('measured input', 'estimated input','measured output', 'estimated output') ;

    % 위상차를 그림 제목으로 출력

    str = sprintf('\\phi_1 = %4.2f[deg], \\phi_2 = %4.2f[deg], \\Delta \\phi = %4.2f[deg]', ePhs1*UNIT_RAD2DEG, ePhs2*UNIT_RAD2DEG, (ePhs2-ePhs1)*UNIT_RAD2DEG) ;
    title(str, 'fontsize', 12)

    Vin = max(ey1)-min(ey1);
    Wout = max(ey2)-min(ey2);
    magnituge = Wout/Vin;

end


%% 배치 폼
clc; clear all; close all;

addpath('modelingData');
%addpath('1차_load1_mag5_freq0.6-1.95');
%addpath('2차_load1_mag5_freq0.6-1.95');

Ts = 0.005; 
volt_to_OMEGADEG = 5/3.3*51*2;

ts = 0.005;
t = 0:ts:6-ts;
Vss = zeros(length(t),1);
GyroOut = zeros(length(t),2);

% fbw = linspace(0.5 ,1.8, 10);
fbw = 0.65: 0.15 : 2.15;

% freq = []; % [Hz]
N = length(fbw);
% 
% file_start = 20;

for i = 1 : N
    str(i)  = sprintf("sindata_freq%.2f.txt",fbw(i));
end 
file_start = 30;
Nstr = length(str);

for i = 1:Nstr
    data(:,:,i) = load(str(i));
end
% 원래 신호  = 2
%  W = 8
%
% column에 저장된 것들 : Wcmd_R, |EncoderWR|, flagR, Vcmd_R, ,

N_DataPackage = length(data(:,1,1));

for i = 1: N
    input(:,1,i) = data(100:1100,2,i);    
    En_R(:,1,i) = data(100:1100,3,i);
end 

% En_R = En_R*volt_to_OMEGADEG;
%En_R = En_R * volt_to_OMEGADEG ;

N_mat = size(En_R);
N_limit = N_mat(1)- 30;

for i = 1:N_mat(3)
    input(:,:,i) = lowpass(input(:,:,i), 40, 1/Ts,'steepness',0.9);
    En_R(:,:,i)  = lowpass(En_R(:,:,i), 40, 1/Ts, 'steepness',0.9);
end 

for i = 1: N_mat(3)
   Hk_R(:,:,i) = [ input(file_start + 3 : N_limit, 1, i) +  2*input(file_start+2 : N_limit-1 ,1, i) + input(file_start+1 :N_limit-2 ,1, i),  ...
                  -En_R(file_start + 2: N_limit-1, 1,i) , - En_R(file_start + 1 : N_limit-2 , 1, i) ];
   ENR_elim(:,:,i) = En_R(file_start +3 : N_limit, 1 , i); % 현재 시점  k 시점
   input_R_elim(:,:,i) = En_R(file_start+1: N_limit-2,1,i); 

end 

N_hk = size(Hk_R);
HkstackR(:,1:3) = Hk_R (:,1:3,1);
ENR_stack(:,1) = ENR_elim(:,1,1);
inputR_stack(:,1) = input_R_elim(:,1,1);

for i = 1:N_hk(3) -1
    HkstackR = cat(1, HkstackR, Hk_R(:,1:3,i+1)); 
    ENR_stack = cat(1, ENR_stack, ENR_elim(:,1,i+1));
    inputR_stack = cat(1, inputR_stack, input_R_elim(:,1,i+1));
end 
N_f = length(ENR_stack);
time = 0:Ts:N_f*Ts -Ts; 

figure , plot(time, ENR_stack); 

P_R = HkstackR' * HkstackR; 
Coef_R = P_R\HkstackR' * ENR_stack;

% Tustin에서의 계수가 나옴
% Tustin --> CT 로의 역 연산을통한 CT 에서의 계수 구하기
Err_R = ENR_stack - HkstackR*Coef_R ;
meanErr_R = mean(sqrt(Err_R.^2)) ; 

b_R  = Coef_R(1); 
a1_R = Coef_R(2);
a0_R = Coef_R(3);

z = tf('z',Ts);
s = tf('s');

Hz_R = (b_R+ 2*b_R*z^-1 + b_R*z^-2) /(1+a1_R*z^-1 + a0_R*z^-2); 

d_R = [-2*b_R*Ts , -b_R*Ts^2,         Ts^2; 
       -2*a1_R*Ts, -a1_R*Ts^2 + 2*Ts^2,  0;
       2*a0_R*Ts+ 2*Ts, Ts^2*a0_R- Ts^2, 0];
   
   
y_R = [4*b_R; 8+4*a1_R; 4-4*a0_R];

coefs_R = d_R\y_R; 

Hs_R = coefs_R(3) / (s^2+coefs_R(1)*s + coefs_R(2))

%%

step(Hs_R)

%%
clear all, close all, clc;
UNIT_RAD2DEG = 180/pi ;
UNIT_DEG2RAD = pi/180 ;

addpath('modelingData');

% f     =  [0.63  0.79 0.81 0.97 1.27 1.43 1.75 1.91 2.07]
fc = 5.0;

f = 0.65: 0.15 : 2.15;

for i = 1 : length(f)
    
    dataName = sprintf('sindata_freq%.2f.txt',f(i));
    File = load(dataName) ;
    
    DEG2RAD = pi/180 ;
    RAD2DEG = 180/pi ;

    time  = File(1:1200,1) ;
    Vcmd  = File(1:1200,2) ;
    %Vgyro = File(:,3) ;
    Omega = File(1:1200,3) ;
    
    coef1 = lsqcurvefit(@(coef1, time) coef1(1)*sin( 2*pi*f(i)*time + coef1(2) ) + coef1(3), [max(Vcmd)  0 (max(Vcmd)+min(Vcmd))/2  ], time, Vcmd) ;
    coef2 = lsqcurvefit(@(coef2, time) coef2(1)*sin( 2*pi*f(i)*time + coef2(2) ) + coef2(3), [max(Omega) 0 (max(Omega)+min(Omega))/2], time, Omega) ;

    eMag1  = coef1(1) ; % estimated magnitude
    ePhs1  = coef1(2) ; % estimated phase
    eBias1 = coef1(3) ; % estimated bias

    eMag2  = coef2(1) ; % estimated magnitude
    ePhs2  = coef2(2) ; % estimated phase
    eBias2 = coef2(3) ; % estimated bias

    ey1 = eMag1 * sin( 2*pi*f(i)*time + ePhs1 ) + eBias1 ; % estimated y
    ey2 = eMag2 * sin( 2*pi*f(i)*time + ePhs2 ) + eBias2 ; % estimated y
    
    Gain        = eMag2 / eMag1 ;
    Phase_Shift = (ePhs2 - ePhs1) * RAD2DEG;
    
    tblOmega(1, i)    = f(i) * (2*pi) ;
    tblMagAtt(1, i)   = Gain ;
    tblPhsDelay(1, i) = Phase_Shift * (DEG2RAD) ;
    
    figure(i)
    %plot(time, ey1, 'g:', 'linewidth', 2), grid on ;
    hold on,plot(time, Vcmd), hold on,plot(time, Omega);
    hold on, plot(time, ey2, 'm:', 'linewidth', 2), grid on ;
    xlabel('time [sec]'), ylabel('estimated [V]'), legend('estimated input', 'estimated output') ;
    str = sprintf('\\phi_1 = %4.2f[deg], \\phi_2 = %4.2f[deg], \\Delta \\phi = %4.2f[deg]', (ePhs1)*UNIT_RAD2DEG, ePhs2*UNIT_RAD2DEG, (ePhs1-ePhs2)*UNIT_RAD2DEG ) ;
    title(str, 'fontsize', 12)
end

tblFreqResp = tblMagAtt .* exp(1i*tblPhsDelay) ;

Nnum = 0 ;
Nden = 2 ;

[num, den]   = invfreqs(tblFreqResp, tblOmega, Nnum, Nden) ;
EstTransFunc = tf(num, den)

[mag,phase,wout] = bode(EstTransFunc);

%%
clear all; close all; clc;

addpath('modelingData');

load("motorModel.mat")
rowdata=load("sindata_freq2.00.txt");

num = EstTransFunc.Numerator{1, 1};
den = EstTransFunc.Denominator{1, 1};

f=2;
t=rowdata(:,1);

valid = sim("modelValidation.slx");

hold on,
plot(valid.tout,valid.modelvalidation,'b','linewidth',2)
plot(rowdata(:,1),rowdata(:,3),'r--','linewidth',2)
grid on,
title("Modeling Validataion"),
xlabel("time[sec]"),ylabel("GryoW[deg/s]")
legend("actual","simulation")
xlim([0 6])

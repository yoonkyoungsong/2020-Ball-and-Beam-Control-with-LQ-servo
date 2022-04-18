clc; close all; clear all;

% time setting
SIM_TIME = struct('time', 0, 'ts', 0.005, ...
                'start', 0, 'final', 10, ...
                'cnt', 1, 'ntime', 0);

SIM_TIME.time = SIM_TIME.start : SIM_TIME.ts : SIM_TIME.final;
SIM_TIME.ntime = length(SIM_TIME.time);

% matrix and inital value
F = [1 SIM_TIME.ts (SIM_TIME.ts)^2;
    0      1       SIM_TIME.ts   ;
    0      0           1         ];
H = [ 1 0 0 ];

X0 = [ 0 0 0 ];


% fmf design parameter & kalman gain
beta = 0.7;
Kf = [1 - beta^3, 1.5*(1-beta)^2*(1+beta)/SIM_TIME.ts, (1-beta)^3/SIM_TIME.ts^2]'; 

% variance of measurement value;
Q = diag([0.1 0.2 0.3]);
R = 0.1; 

% buffer
pXk = zeros(3,SIM_TIME.ntime);
eXk = zeros(3,SIM_TIME.ntime);
yk = zeros(1,SIM_TIME.ntime);
dffk = zeros(1,SIM_TIME.ntime);
Rdl = zeros(1,SIM_TIME.ntime);

for idx = 1 : SIM_TIME.ntime
    
    % GENERATE NOISE
    wk = sqrt(Q)*randn(3, 1);
    vk = sqrt(R)*randn(1, 1);
    
    % SYSTEM PROPAGATION
    if ( idx == 1 )
        pXk(:,idx) = F*X0' + wk;
        dffk(idx) = (pXk(1,idx) - X0(1)) / SIM_TIME.ts;
    else
        pXk(:,idx) = F*pXk(:,idx-1) + wk;
        dffk(idx) = (pXk(1,idx) - pXk(1,idx-1)) / SIM_TIME.ts;
    end
    
    % MEASUREMENT UPATE
    yk(idx) = H*pXk(:,idx) + vk;
    
    % ESTIMATION
    eXk(:,idx) = pXk(:,idx) + Kf*(yk(idx) - H*pXk(:,idx));
    pXk(:,idx) = eXk(:,idx);
    
    % RESIDUAL
    Rdl(:,idx) = yk(idx) - H*pXk(:,idx);
    
end
%%


figure,
plot(SIM_TIME.time, pXk,'linewidth',2); hold on; 
plot(SIM_TIME.time, yk,'r--','linewidth',2);
xlabel('time [sec]'); ylabel('state [-]');
legend('p pos','p vel','p acc','yk');
title('estimated state and measurement');

figure,
plot(SIM_TIME.time, pXk(2,:),'linewidth',2); hold on; 
plot(SIM_TIME.time, dffk,'r--','linewidth',2);
xlabel('time [sec]'); ylabel('vel [m/s]');
legend('est vel','diff pos');
title('COMPARE WITH FMF OF VEL AND DIFF POS');

figure,
plot(SIM_TIME.time, Rdl,'linewidth',2);
xlabel('time [sec]'); ylabel('\inc pos [m]');
ylim = ([-0.1 0.1]);
legend('Residual');

%%
close all; clc;

beta = 0.7;

Kf = [1 - beta^3, 1.5*(1-beta)^2*(1+beta)/SIM_TIME.ts, (1-beta)^3/SIM_TIME.ts^2]'; 

z = tf('z',SIM_TIME.ts);

mff_tf = (F*Kf)'*inv( z*eye(3,3) - F*( eye(3,3) - Kf*H) );

figure,
subplot(3,1,1),
step (mff_tf(1)) ;
subplot(3,1,2),
step (mff_tf(2)) ;
subplot(3,1,3),
step (mff_tf(3)) ;

figure,
subplot(3,1,1),
bode (mff_tf(1)) ;
subplot(3,1,2),
bode (mff_tf(2)) ;
subplot(3,1,3),
bode (mff_tf(3)) ;

%% fading memory filter
clear all; close all; clc;

beta = 0.7;
T=0.0005;
time=0:T:5;
f=0.5;

Xkbar=zeros(3,1);

yk=sin(2*pi*f*time);

F=[1 T 0.5*T^2; 0 1 T; 0 0 1];
H=[0 0 1];
kf=[1-beta^3; (1.5*(1-beta)^2*(1+beta))/T; (1-beta)^3/T^2]

Xkhat=Xkbar+kf*(yk-H*Xkbar);
Xkbar=F*Xkhat;

plot(time,yk)
hold on
plot(time,Xkhat(1,:))


%%

for i=0.2:0.2:1

    beta=i;
    Xkbar=zeros(3,1);

    yk=sin(2*pi*f*time);

    F=[1 T 0.5*T^2; 0 1 T; 0 0 1];
    H=[0 0 1];
    kf=[1-beta^3; (1.5*(1-beta)^2*(1+beta))/T; (1-beta)^3/T^2]

    Xkhat=Xkbar+kf*(yk-H*Xkbar);
    Xkbar=F*Xkhat;

    hold on
    plot(time,Xkhat(1,:))
    leg(i)=sprintf("beta%f",i)
    legend(leg)

end

plot(time,yk)
function Vss = CoeffLinearization(Vcmd)

    %CW
    cwVcmdDead=0.05;
    cwVssDead =1.3;

    cwVcmdSatuate=4.4325;
    cwVssSatuate=2.67;

    cwVcmdMid = 2.9545;
    cwVssMid = 2.09;


    % CCW
    ccwVcmdDead=-0.05;
    ccwVssDead=-1.3;

    ccwVcmdSatuate=-4.3470;
    ccwVssSatuate=-2.625;

    ccwVcmdMid = -2.4163;
    ccwVssMid = -1.91;
    
    
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

    Vss = zeros(length(Vcmd),1);

    for i = 1 : length(Vcmd)
        if (Vcmd(i) <= -5) 
            Vcmd(i) = -5 ;
            Vss(i)=ccwCoeff(1)*(Vcmd(i))^2+ccwCoeff(2)*(Vcmd(i))+ccwCoeff(3);

        elseif (Vcmd(i) <= -0.05 && Vcmd(i) > -5) 
            Vss(i)=ccwCoeff(1)*(Vcmd(i))^2+ccwCoeff(2)*(Vcmd(i))+ccwCoeff(3);

        elseif (Vcmd(i) >= 0.05 && Vcmd(i) < 5) 
            Vss(i)=cwCoeff(1)*(Vcmd(i))^2+cwCoeff(2)*(Vcmd(i))+cwCoeff(3);

        elseif ( Vcmd(i) >= 5) 
            Vcmd(i)=5;
            Vss(i)=cwCoeff(1)*(Vcmd(i))^2+cwCoeff(2)*(Vcmd(i))+cwCoeff(3);
        else
            Vss(i) = 0;
        end
    end
end


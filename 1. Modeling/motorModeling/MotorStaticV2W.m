function Wout = MotorStaticV2W(Vcmd)

 Wout = zeros(length(Vcmd),1);

       a1 =       95.93;
       b1 =       1.311;
       c1 =      -3.165;
       a2 =       151.7;
       b2 =      0.6794;
       c2 =   -0.006173;
       a3 =       -2267;
       b3 =       2.719;
       c3 =       -2.87;
       a4 =       5.688;
       b4 =       4.255;
       c4 =    0.006059;
       a5 =        2265;
       b5 =       2.718;
       c5 =      -2.869;
       a6 =       16.71;
       b6 =       5.691;
       c6 =       3.136;
       a7 =       18.32;
       b7 =       5.958;
       c7 =    -0.01675;
       a8 =       9.724;
       b8 =       6.466;
       c8 =      -3.169;


    for i=1:length(Vcmd)
        if (Vcmd(i)>=3.4)
            Vcmd(i)=3.4;
        elseif (Vcmd(i)<=-3.4)
            Vcmd(i)=-3.4;
        end
        Wout(i) =  a1*sin(b1*Vcmd(i)+c1) + a2*sin(b2*Vcmd(i)+c2) + a3*sin(b3*Vcmd(i)+c3)+ a4*sin(b4*Vcmd(i)+c4)+ a5*sin(b5*Vcmd(i)+c5)+ a6*sin(b6*Vcmd(i)+c6)+ a7*sin(b7*Vcmd(i)+c7)+ a8*sin(b8*Vcmd(i)+c8);
        
        if(Vcmd(i)>=-1.3 && Vcmd(i)<=1.3)
            Wout(i)=0;
        end
        
    end

end
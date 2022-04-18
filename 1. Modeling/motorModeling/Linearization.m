function Vss = Linearization(Vcmd, cwCoeff, ccwCoeff)

    Vss = zeros(length(Vcmd),1);

    for i = 1 : length(Vcmd)
        if (Vcmd(i) <= -5) 
            Vcmd(i) = -5 ;
            Vss(i)=ccwCoeff(1)*(Vcmd(i))^2+ccwCoeff(2)*(Vcmd(i))+ccwCoeff(3);

        elseif (Vcmd(i) <= -0.5 && Vcmd(i) > -5) 
            Vss(i)=ccwCoeff(1)*(Vcmd(i))^2+ccwCoeff(2)*(Vcmd(i))+ccwCoeff(3);

        elseif (Vcmd(i) >= 0.5 && Vcmd(i) < 5) 
            Vss(i)=cwCoeff(1)*(Vcmd(i))^2+cwCoeff(2)*(Vcmd(i))+cwCoeff(3);

        elseif ( Vcmd(i) >= 5) 
            Vcmd(i)=5;
            Vss(i)=cwCoeff(1)*(Vcmd(i))^2+cwCoeff(2)*(Vcmd(i))+cwCoeff(3);
        else
            Vss(i) = 0;
        end
    end
end


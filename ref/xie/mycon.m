function [c,ceq]= mycon(x)
    

    ceq(1)= x(4)*x(7)+x(5)*x(8)+x(6)*x(9); 
    ceq(2)= x(4)^2+x(5)^2+x(6)^2-1;
    ceq(3)= x(7)^2+x(8)^2+x(9)^2-1;
    c=[];


end
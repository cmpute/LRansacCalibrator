function f = myfun (x,lx,ly)
   
   
    f=0;

   

    for i = 1:1:size(lx,2)
        a = lx(1,i)-x(1);
        b = lx(2,i)-x(2);
        c = lx(3,i)-x(3);
        err = (a*x(4)+b*x(5)+c*x(6))/(a^2+b^2+c^2)^0.5 ;
        
        if (err >=0 )
            f =  f +  err;
        else 
            f =  f -  1;
        end
    end
    for i = 1:1:size(ly,2)

        a = ly(1,i)-x(1);
        b = ly(2,i)-x(2);
        c = ly(3,i)-x(3);
        err = (a*x(7)+b*x(8)+c*x(9))/(a^2+b^2+c^2)^0.5 ;
        if (err >=0 )
            f =  f +  err;
         else 
            f =  f -  1;
         end  
    end
    f=-f;
end


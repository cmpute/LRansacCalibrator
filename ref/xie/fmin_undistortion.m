function loss = fmin_undistortion (P,u_x_,u_y_,Ins_para)
        u_x=P(1);
        u_y=P(2);   
% 
%     fx=Ins_para(1);
%     fy=Ins_para(2);
%     u0= Ins_para(3);
%     v0= Ins_para(4);

    k1 = Ins_para(5);
    k2 = Ins_para(6);
    p1 = Ins_para(7);
    p2 = Ins_para(8);
    
            r_2 = u_x.^2+u_y.^2;
            loss = norm(u_x.*(1+k1*r_2+k2*r_2.^2)+(2*p1*u_x.*u_y+p2*(r_2+2*u_x.^2))-u_x_,2)+norm(u_y.*(1+k1*r_2+k2*r_2.^2)+(2*p2*u_x.*u_y+p1*(r_2+2*u_y.^2))-u_y_,2);
end

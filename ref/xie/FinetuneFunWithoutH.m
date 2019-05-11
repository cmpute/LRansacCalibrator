function loss = FinetuneFunWithoutH(H,P,Points_c_final,Points_l_final,resolution,mode,proportion)
 num = size(Points_c_final,1);
 HH = [H(1:4)';H(5:8)';H(9:12)'];
 
 u0 = P(1)*proportion(1);
 v0 = P(2)*proportion(2);
 
 k1 = P(3)*proportion(3);
 k2 = P(4)*proportion(4);
 
 p1 = P(5)*proportion(5);
 p2 = P(6)*proportion(6);
%  u0 = P(1);
%  v0 = P(2);
%  
%  k1 = P(3);
%  k2 = P(4);
%  
%  p1 = P(5);
%  p2 = P(6);




 
 ProjectPoints = HH *[Points_l_final';ones(1,size(Points_l_final',2))];
 ProjectPoints(1,:)=ProjectPoints(1,:)./ProjectPoints(3,:);
 ProjectPoints(2,:)=ProjectPoints(2,:)./ProjectPoints(3,:);
 ProjectPoints(3,:)=ProjectPoints(3,:)./ProjectPoints(3,:);
 
 u_x=ProjectPoints(1,:)-u0;  %转到相机坐标
 u_y=ProjectPoints(2,:)-v0;  
        %考虑畸变
        r_2 = u_x.^2+u_y.^2;
        u_x_ = u_x.*(1+k1*r_2+k2*r_2.^2)+(2*p1*u_x.*u_y+p2*(r_2+2*u_x.^2));
        u_y_ = u_y.*(1+k1*r_2+k2*r_2.^2)+(2*p2*u_x.*u_y+p1*(r_2+2*u_y.^2));
        
        u_x = u_x_ +u0;   %转回像素坐标
        u_y = u_y_ +v0; 
        
        err = Points_c_final' - [u_x;u_y];
        lmd = 0.001;
        loss =0;
        for i = 1:1:num
            loss = loss+norm(err(:,i),2)^2;
        end
        if mode==1
            loss =loss/num+lmd*((u0-resolution(1)/2)^2+(v0-resolution(2)/2)^2);
        
        else 
            loss = (loss/num)^0.5;
        end

%         loss = (norm(err(1,:),2)^2 + norm(err(2,:),2)^2)/length(u_x)+lmd*((u0-resolution(1)/2)^2+(v0-resolution(2)/2)^2);


end
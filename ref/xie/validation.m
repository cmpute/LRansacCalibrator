HH = [Calib_Re(1:4)';Calib_Re(5:8)';Calib_Re(9:12)'];
 
 u0 = Calib_Re(13);
 v0 = Calib_Re(14);
 
 k1 = Calib_Re(15);
 k2 = Calib_Re(16);
 
 p1 = Calib_Re(17);
 p2 = Calib_Re(18);
 
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
        
       
        
plot(Points_c_final(:,1),Points_c_final(:,2),'ro','MarkerSize',10)
hold on
plot(u_x,u_y,'b*','MarkerSize',10)
hold on

plot(ProjectPoints(1,:),ProjectPoints(2,:),'b*','MarkerSize',10)
hold on

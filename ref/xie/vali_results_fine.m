function points_3d_image=vali_results_fine(points_3d,Calib_Re,points_3d_dis,points_3d_int,resolution,flag)



HH = [Calib_Re(1:4)';Calib_Re(5:8)';Calib_Re(9:12)'];
 
 u0 = Calib_Re(13);
 v0 = Calib_Re(14);
 
 k1 = Calib_Re(15);
 k2 = Calib_Re(16);
 
 p1 = Calib_Re(17);
 p2 = Calib_Re(18);
 
 ProjectPoints = HH *[points_3d;ones(1,size(points_3d,2))];
 ProjectPoints(1,:)=ProjectPoints(1,:)./ProjectPoints(3,:);
 ProjectPoints(2,:)=ProjectPoints(2,:)./ProjectPoints(3,:);
 ProjectPoints(3,:)=ProjectPoints(3,:)./ProjectPoints(3,:);
 

 
  %去掉非图片上点
  
  outPoints = (ProjectPoints(1,:)<-100 | ProjectPoints(2,:)<-100 | ProjectPoints(1,:)>resolution(1)+100 | ProjectPoints(2,:)>resolution(2)+100);
  
  points_3d_image=points_3d(:,~outPoints);
  ProjectPoints(:,outPoints)=[];
  points_3d_dis(:,outPoints)=[];
  points_3d_int(:,outPoints)=[];
  
  u_x=ProjectPoints(1,:)-u0;  %转到相机坐标
  u_y=ProjectPoints(2,:)-v0;  
 

        %考虑畸变
        r_2 = u_x.^2+u_y.^2;
        u_x_ = u_x.*(1+k1*r_2+k2*r_2.^2)+(2*p1*u_x.*u_y+p2*(r_2+2*u_x.^2));
        u_y_ = u_y.*(1+k1*r_2+k2*r_2.^2)+(2*p2*u_x.*u_y+p1*(r_2+2*u_y.^2));
        
        u_x = u_x_ +u0;   %转回像素坐标
        u_y = u_y_ +v0;
        
 dis_color = points_3d_dis/max(points_3d_dis)*255;
int_color = points_3d_int/max(points_3d_int)*255;
color_map = jet(256);
color_map=flipud(color_map);
        
if flag == 1
for i = 1:1:255
plot (u_x((dis_color>i-1 & dis_color<i)), u_y((dis_color>i-1 & dis_color<i))...
                                  ,'*' ,'MarkerSize',1,'MarkerEdgeColor',color_map(i,:))
    hold on; 
end
end

if flag == 0
for i = 1:1:255
plot ( u_x((int_color>i-1 & int_color<i)), u_y((int_color>i-1 & int_color<i))...
                                  ,'*' ,'MarkerSize',1,'MarkerEdgeColor',color_map(i,:))
    hold on; 
end
end
end

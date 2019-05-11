function [projection_val_points,lidar_val_points]=gener_val_points(Ins_para,s,resolution,R_c2l,T_c2l)


    fx=Ins_para(1);
    fy=Ins_para(2);
    u0= Ins_para(3);
    v0= Ins_para(4);

    k1 = Ins_para(5);
    k2 = Ins_para(6);
    p1 = Ins_para(7);
    p2 = Ins_para(8);
 
    T = [fx,0,u0;0,fy,v0;0,0,1];
 

 u_x_s = 100:50:resolution(1)-100;
 u_y_s = 100:30:resolution(2)-100;
 [u_x_s,u_y_s]=meshgrid(u_x_s,u_y_s);
 u_x_s =reshape(u_x_s,1,[]); 
 u_y_s =reshape(u_y_s,1,[]);
 projection_val_points = [u_x_s;u_y_s;ones(1,length(u_x_s))];
 
 lidar_val_points_root = T\ projection_val_points;
 
 
 lidar_val_points = cell(1,length(s));
 for i =1:1:length(s)
    lidar_val_points{i}=s(i)*lidar_val_points_root;
    lidar_val_points{i}=(R_c2l*lidar_val_points{i}+T_c2l)';%将相机坐标下的坐标转到lidar坐标系
 end
 
     u_x=u_x_s-u0;  %转到相机坐标
     u_y=u_y_s-v0;  


            %考虑畸变
            r_2 = u_x.^2+u_y.^2;
            u_x_ = u_x.*(1+k1*r_2+k2*r_2.^2)+(2*p1*u_x.*u_y+p2*(r_2+2*u_x.^2));
            u_y_ = u_y.*(1+k1*r_2+k2*r_2.^2)+(2*p2*u_x.*u_y+p1*(r_2+2*u_y.^2));
            projection_val_points(1,:) = u_x_ +u0;   %转回像素坐标
            projection_val_points(2,:) = u_y_ +v0;
            
            projection_val_points=projection_val_points';
            projection_val_points(:,3)=[];
end
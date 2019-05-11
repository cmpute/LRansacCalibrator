    fx=Ins_para(1);
    fy=Ins_para(2);
    u0= Ins_para(3);
    v0= Ins_para(4);

    k1 = Ins_para(5);
    k2 = Ins_para(6);
    p1 = Ins_para(7);
    p2 = Ins_para(8);
 
    T = [fx,0,u0;0,fy,v0;0,0,1];
%      HHH=T*[R_l2c,-Loc_c2l];
    HHH=T*RT_P;
    
    for i = 1:1:size(Points_l_final,1)
 
        project_by_calib(:,i)=HHH*[Points_l_final(i,:)';1];
        
              
 
end  
project_by_calib(1,:)=project_by_calib(1,:)./project_by_calib(3,:);
project_by_calib(2,:)=project_by_calib(2,:)./project_by_calib(3,:);
project_by_calib(3,:)=project_by_calib(3,:)./project_by_calib(3,:);
    
for j =1:1:num_cal
number=(j-1)*num_x*num_y;
    figure();
    




plot(project_by_calib(1,1+number:36+number),project_by_calib(2,1+number:36+number),'.')
hold on
plot(projection_val_points_undistortion(:,1),projection_val_points_undistortion(:,2),'o')
hold on
% plot(Points_c_final_undistortion(1+number:36+number,1),Points_c_final_undistortion(1+number:36+number,2),'o')
% hold on
% plot(Points_c_final(1:36,1),Points_c_final(1:36,2),'o')
% hold on

axis equal;
axis([0 resolution(1) 0 resolution(2)]);

end

%%
H(1:12)=[HHH(1,1:4)';HHH(2,1:4)';HHH(3,1:4)'];


sk = 1/(HH(3,1)^2+HH(3,2)^2+HH(3,3)^2)^0.5;
H(1:12)=H(1:12)*sk;

%%

 HH = [Calib_Re(1:4)';Calib_Re(5:8)';Calib_Re(9:12)'];
 
 u0 = Calib_Re(13);
 v0 = Calib_Re(14);
 
 k1 = Calib_Re(15);
 k2 = Calib_Re(16);
 
 p1 = Calib_Re(17);
 p2 = Calib_Re(18);

    
    
        %投影
for i = 1:1:36
 
        project_by_calib(:,i)=HH*[Points_l_final(i,:)';1];
        
              
 
end  

    project_by_calib(1,:) = project_by_calib(1,:)./ project_by_calib(3,:);
    project_by_calib(2,:) = project_by_calib(2,:)./ project_by_calib(3,:);
    project_by_calib(3,:) = project_by_calib(3,:)./ project_by_calib(3,:);%实际像素坐标系位置

     u_x=project_by_calib(1,:)-u0;  %转到相机坐标
     u_y=project_by_calib(2,:)-v0;  


            %考虑畸变
            r_2 = u_x.^2+u_y.^2;
            u_x_ = u_x.*(1+k1*r_2+k2*r_2.^2)+(2*p1*u_x.*u_y+p2*(r_2+2*u_x.^2));
            u_y_ = u_y.*(1+k1*r_2+k2*r_2.^2)+(2*p2*u_x.*u_y+p1*(r_2+2*u_y.^2));
            project_by_calib(1,:) = u_x_ +u0;   %转回像素坐标
            project_by_calib(2,:) = u_y_ +v0;
            
            
plot(project_by_calib(1,:),project_by_calib(2,:),'.')
hold on

plot(Points_c_final(1:36,1),Points_c_final(1:36,2),'o')
hold on

axis equal;
axis([0 resolution(1) 0 resolution(2)]);

%%
 fx=Ins_para(1);
    fy=Ins_para(2);
    u0= Ins_para(3);
    v0= Ins_para(4);

    k1 = Ins_para(5);
    k2 = Ins_para(6);
    p1 = Ins_para(7);
    p2 = Ins_para(8);
 
    T = [fx,0,u0;0,fy,v0;0,0,1];
    HHH=T*[R_l2c,-Loc_c2l];
    aaaa=R_l2c*lidar_val_points{1}'-Loc_c2l;
    
    

 
       
      [project_points,image_points,T]=project(aaaa,Ins_para);
              
      for i = 1:1:size(lidar_val_points{1},1)
 
        project_by_calib(:,i)=HHH*[lidar_val_points{1}(i,:)';1];
        
              
 
      end  
project_by_calib(1,:)=project_by_calib(1,:)./project_by_calib(3,:);
project_by_calib(2,:)=project_by_calib(2,:)./project_by_calib(3,:);
project_by_calib(3,:)=project_by_calib(3,:)./project_by_calib(3,:);
        
    

    




% plot(project_points(1,:),project_points(2,:),'*')
% hold on

plot(project_by_calib(1,:),project_by_calib(2,:),'*')
hold on

% plot(projection_val_points(:,1),projection_val_points(:,2),'o')
% hold on

plot(projection_val_points_undistortion(:,1),projection_val_points_undistortion(:,2),'o')
hold on

% plot(Points_c_final_undistortion(1+number:36+number,1),Points_c_final_undistortion(1+number:36+number,2),'o')
% hold on
% plot(Points_c_final(1:36,1),Points_c_final(1:36,2),'o')
% hold on

axis equal;
axis([0 resolution(1) 0 resolution(2)]);


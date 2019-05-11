function []=DrawConfigure(CORNER_POINTS_source,PROJECTION_POINTS_source,num_cal,num_x,num_y,Loc_cam,R_c2w,Loc_c2l,R_l2c,T_l2c)

Points_l_final_source = [];
Points_c_final_source =[];
for i=1:1:num_cal
    Points_l_final_source = [Points_l_final_source;CORNER_POINTS_source{i}'];
    Points_c_final_source = [Points_c_final_source;PROJECTION_POINTS_source{i}'];
end

mida= [R_l2c,T_l2c]*[Points_l_final_source';ones(1,size(Points_l_final_source,1))];
Points_l_final_source=mida(1:3,:)';

start_P_l = zeros(4,(num_x+num_y)*num_cal);
end_P_l = zeros(4,(num_x+num_y)*num_cal);
start_P_c = zeros(3,(num_x+num_y)*num_cal);
end_P_c = zeros(3,(num_x+num_y)*num_cal);

n=1;
for j =1:1:num_cal
        number =(j-1)*num_x*num_y;
        m=1;
    for i =1+number:1:num_y+number
        start_P_l(:,n)=[Points_l_final_source(i,:)';1];
        end_P_l(:,n)=[Points_l_final_source(i+(num_x-1)*num_y,:)';1];
        start_P_c(:,n)=[Points_c_final_source(i,:)'];
        end_P_c(:,n)=[Points_c_final_source(i+(num_x-1)*num_y,:)'];
        n=n+1;
    end
    for i =1+number:1:num_x+number
        start_P_l(:,n)=[Points_l_final_source((m-1)*num_y+1+number,:)';1];
        end_P_l(:,n)=[Points_l_final_source(m*num_y+number,:)';1];
        start_P_c(:,n)=[Points_c_final_source((m-1)*num_y+1+number,:)'];
        end_P_c(:,n)=[Points_c_final_source(m*num_y+number,:)'];
        n=n+1;
        m=m+1;
    end
end




map = jet(num_cal);
figure()
for j =1:1:num_cal
        
    number =(j-1)*(num_x+num_y);
       
    for i =1+number:1:num_x+num_y+number
       line  = [start_P_l(1,i),end_P_l(1,i);start_P_l(2,i),end_P_l(2,i);start_P_l(3,i),end_P_l(3,i)];
       plot3(line(1,:),line(2,:),line(3,:),'Color',map(j,:))
       hold on
    end
        
        
    
end
%draw camera 
DrawCamera(Loc_cam,R_c2w);
%draw lidar
DrawLidar(Loc_cam,R_c2w,Loc_c2l,R_l2c)




grid on
axis equal
xlabel('x')
ylabel('y')
zlabel('z')

figure()
for j =1:1:num_cal
        
    number =(j-1)*(num_x+num_y);
       
    for i =1+number:1:num_x+num_y+number
       line2  = [start_P_c(1,i),end_P_c(1,i);start_P_c(2,i),end_P_c(2,i)];
       plot(line2(1,:),line2(2,:),'Color',map(j,:))
       hold on
    end
        
   
    
end
grid on
axis equal   
end
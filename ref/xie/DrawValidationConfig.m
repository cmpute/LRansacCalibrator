function DrawValidationConfig(Ins_para,s,resolution,num_cal,Loc_cam,R_c2w,Loc_c2l,R_l2c)


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
 num_x = length(u_x_s);
num_y = length(u_y_s);
 
 [u_x_s,u_y_s]=meshgrid(u_x_s,u_y_s);
 u_x_s =reshape(u_x_s,1,[]); 
 u_y_s =reshape(u_y_s,1,[]);
 projection_val_points = [u_x_s;u_y_s;ones(1,length(u_x_s))];
 
 val_points_root = T\ projection_val_points;
 
 
 val_points = cell(1,length(s));
 for i =1:1:length(s)
    val_points{i}=s(i)*val_points_root;
   
 end
 
 Points_l_final_source = [];

for i=1:1:num_cal
    Points_l_final_source = [Points_l_final_source;val_points{i}'];
 
end





start_P_l = zeros(4,(num_x+num_y)*num_cal);
end_P_l = zeros(4,(num_x+num_y)*num_cal);


n=1;
for j =1:1:num_cal
        number =(j-1)*num_x*num_y;
        m=1;
    for i =1+number:1:num_y+number
        start_P_l(:,n)=[Points_l_final_source(i,:)';1];
        end_P_l(:,n)=[Points_l_final_source(i+(num_x-1)*num_y,:)';1];

        n=n+1;
    end
    for i =1+number:1:num_x+number
        start_P_l(:,n)=[Points_l_final_source((m-1)*num_y+1+number,:)';1];
        end_P_l(:,n)=[Points_l_final_source(m*num_y+number,:)';1];

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
    
    plot3 (Points_l_final_source((j-1)*num_x*num_y+1:j*num_x*num_y,1),Points_l_final_source((j-1)*num_x*num_y+1:j*num_x*num_y,2)...
        ,Points_l_final_source((j-1)*num_x*num_y+1:j*num_x*num_y,3),'*','Color',map(j,:),'MarkerSize',log(s(j)))
        
    
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
 
 
   
end
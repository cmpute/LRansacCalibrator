function simresults = simulate(par_num)


%% define sensor position
Loc_cam = [0,0,0]';%相机在世界坐标下的坐标
Ori_cam = [0,0,0]';

Loc_c2l = [-0.5+1*rand;-0.5+1*rand;-0.5+1*rand]; %实际上是负的lidar在相机坐标系下的坐标
Ori_c2l = [rand;rand;rand];

R_w2c = rodrigues(Ori_cam);
R_c2l = rodrigues(Ori_c2l);
T_c2l = R_c2l*Loc_c2l;


R_c2w = R_w2c';
R_l2c = R_c2l';
T_l2c = -Loc_c2l;
%% generate camera and pattern
%parttern parameters
num_x = 6;
num_y = 6;
ddx = 0.2;
ddy = 0.2;
%
%camera parameters
resolution = [1024,512];
level = 0.4;
%
flag =1;
while(flag) %循环用来保证相机的图像在视野中
    %% in_Para

    Ins_para=gener_cam(resolution,level);
    
   
    
    [Loc_par,R_w2p,R_p2w,X_,Y_,Z_,corner_points_ ]=gener_par(num_x,num_y,ddx,ddy);
    corner_points_=R_w2c*(corner_points_-Loc_cam);%将世界坐标下标定板的坐标转到相机坐标系
      [project_points,image_points,T_projection]=project(corner_points_,Ins_para);
    


            if(project_points(1,:)>0 & project_points(1,:)<resolution(1) ...
                    & project_points(2,:)>0 & project_points(2,:)<resolution(2)  )
                flag =0;
            end
           
       
end

%generate pattern for calbration

num_cal = par_num;
CORNER_POINTS = cell(1,num_cal);
PROJECTION_POINTS = cell(1,num_cal);
%手动指定标定板位置
% par_location(:,1) = [-1;0;5];
% par_location(:,2) = [0;0;5];
% par_location(:,3) = [1;0;5];
% par_location(:,4) = [-5;1;15];
% par_location(:,5) = [5;-1;15];

for i = 1:1:num_cal
    flag = 1;
    while(flag) %循环用来保证相机的图像在视野中
        
        [Loc_par,R_w2p,R_p2w,X_,Y_,Z_,corner_points_ ]=gener_par(num_x,num_y,ddx,ddy);
%         [Loc_par,R_w2p,R_p2w,X_,Y_,Z_,corner_points_ ]=gener_par(num_x,num_y,ddx,ddy,par_location(:,i));

        corner_points_=R_w2c*(corner_points_-Loc_cam);%将世界坐标下标定板的坐标转到相机坐标系
        [project_points,image_points,T_projection]=project(corner_points_,Ins_para);

 

                if(project_points(1,:)>0 & project_points(1,:)<resolution(1) ...
                        & project_points(2,:)>0 & project_points(2,:)<resolution(2)  )
                    flag =0;
                end

    end
    corner_points_=R_c2l*corner_points_+T_c2l;%将相机坐标下标定板的坐标转到lidar坐标系
    CORNER_POINTS{i}=corner_points_;
    PROJECTION_POINTS{i}=project_points;
end
    
    
%add noice
level = 3;
CORNER_POINTS_source=CORNER_POINTS;%without noise
PROJECTION_POINTS_source=PROJECTION_POINTS;%without noise
[CORNER_POINTS,PROJECTION_POINTS]=add_noice(CORNER_POINTS,PROJECTION_POINTS,level); 


%% plot calibration pattern location

% DrawConfigure(CORNER_POINTS_source,PROJECTION_POINTS_source,num_cal,num_x,num_y,Loc_cam,R_c2w,Loc_c2l,R_l2c,T_l2c)
% axis([0 resolution(1) 0 resolution(2)]);
%% our method
Points_l_final = [];
Points_c_final = [];
for i=1:1:num_cal
    Points_l_final = [Points_l_final;CORNER_POINTS{i}'];
    Points_c_final = [Points_c_final;PROJECTION_POINTS{i}(1:2,:)'];

end

[HH,H]=solv_the_initial_H_4(Points_c_final,Points_l_final);


initial_H = H(1:12);
%fine tune without H
initial_P = [resolution(1)/2;resolution(2)/2;0;0;0;0];
proportion = ones(6,1);

min_fun=@(P)FinetuneFunWithoutH(initial_H,P,Points_c_final,Points_l_final,resolution,1,proportion);
initial_P = fminsearch(min_fun,initial_P,optimset('MaxFunEvals',40000,'MaxIter',40000,...
                           'Algorithm','levenberg-marquardt','ToLX',1e-6));%'Display','iter',
                       
%对每个变量进行近似归一化                       
proportion=initial_P;
min_fun=@(P)FinetuneFunWithoutH(initial_H,P,Points_c_final,Points_l_final,resolution,1,proportion);
initial_P = fminsearch(min_fun,ones(6,1),optimset('MaxFunEvals',40000,'MaxIter',40000,...
                            'Algorithm','levenberg-marquardt','ToLX',1e-6));%'Display','iter',
% P=initial_P.*proportion;


proportion_ =[initial_H;proportion.*initial_P];
min_fun = @(P) FinetuneFun(P,Points_c_final,Points_l_final,resolution,1,proportion_);
loss_min = 10000;
rand_value = ones(18,1);
for i=1:1:30
   
    Calib_Re = fminsearch(min_fun,rand_value,optimset('MaxFunEvals',40000,'MaxIter',40000,...
                            'Algorithm','levenberg-marquardt','ToLX',1e-6));%'Display','iter',
    
                        
%       loss = Loss_of_Our_Method(Calib_Re,Points_c_final,Points_l_final,resolution,0);
                
                        
                        
      loss = FinetuneFun(Calib_Re,Points_c_final,Points_l_final,resolution,0,proportion_);
    if loss<loss_min
        loss_min=loss;
        Calib_Re_final = Calib_Re;
    end
     
     rand_value = (0.8+0.4*rand(18,1)).*Calib_Re_final;  
     i            
end
Calib_Re = Calib_Re_final.*proportion_;
loss_withnoise = FinetuneFun(Calib_Re,Points_c_final,Points_l_final,resolution,0);


Points_l_final_source = [];
Points_c_final_source= [];
for i=1:1:num_cal
    Points_l_final_source = [Points_l_final_source;CORNER_POINTS_source{i}'];
    Points_c_final_source = [Points_c_final_source;PROJECTION_POINTS_source{i}(1:2,:)'];

end
loss_withoutnoise = FinetuneFun(Calib_Re, Points_c_final_source,Points_l_final_source ,resolution,0);

% %loss of location and rotation
% HH = [Calib_Re(1:4)';Calib_Re(5:8)';Calib_Re(9:12)'];
% 
% RT = T_projection\HH;
% 
% sk=3/(norm(RT(:,1),2)+norm(RT(:,2),2)+norm(RT(:,3),2));


% sk = 1/(RT(3,1)^2+RT(3,2)^2+RT(3,3)^2)^0.5;
% RT=RT*sk;
% loss_location = norm(RT(:,4)-T_l2c,2);


%% P.Moghadam method
%ins_para without noise
Ins_para_P = Ins_para;
Ins_para_P_nodistortion =Ins_para_P;%畸变参数为0的相机内参，为了直接用project函数
Ins_para_P_nodistortion(5:8)=0;

%undistortion
Points_c_final_undistortion=undistortion(Points_c_final,Ins_para_P,resolution); 
%cal
Tht=P_method_cal(Points_c_final_undistortion,Points_l_final,num_x,num_y,num_cal,Ins_para_P_nodistortion);

%ins_para with noise
Ins_para_P_noise = Ins_para_P*(0.98+0.04*rand);%add noise
Ins_para_P_noise_nodistortion =Ins_para_P_noise;%畸变参数为0的相机内参，为了直接用project函数
Ins_para_P_noise_nodistortion(5:8)=0;
%undistortion
Points_c_final_undistortion_insnoised=undistortion(Points_c_final,Ins_para_P_noise,resolution); 
%cal
Tht_withnoise=P_method_cal(Points_c_final_undistortion_insnoised,Points_l_final,num_x,num_y,num_cal,Ins_para_P_noise_nodistortion);




% l_P_c = zeros((num_x+num_y)*num_cal,3);
% start_P_l = zeros(4,(num_x+num_y)*num_cal);
% end_P_l = zeros(4,(num_x+num_y)*num_cal);
% start_P_c = zeros(3,(num_x+num_y)*num_cal);
% end_P_c = zeros(3,(num_x+num_y)*num_cal);
% n=1;
% for j =1:1:num_cal
%         number =(j-1)*num_x*num_y;
%         m=1;
%     for i =1+number:1:num_y+number
%         l_P_c(n,:)=(cross([Points_c_final_undistortion(i,:)';1],[Points_c_final_undistortion(i+(num_x-1)*num_y,:)';1]))';
%         start_P_l(:,n)=[Points_l_final(i,:)';1];
%         end_P_l(:,n)=[Points_l_final(i+(num_x-1)*num_y,:)';1];
%         start_P_c(:,n)=[Points_c_final_undistortion(i,:)';1];
%         end_P_c(:,n)=[Points_c_final_undistortion(i+(num_x-1)*num_y,:)';1];
%         n=n+1;
%     end
%     for i =1+number:1:num_x+number
%         l_P_c(n,:)=(cross([Points_c_final_undistortion((m-1)*num_y+1+number,:)';1],[Points_c_final_undistortion(m*num_y+number,:)';1]))';
%         start_P_l(:,n)=[Points_l_final((m-1)*num_y+1+number,:)';1];
%         end_P_l(:,n)=[Points_l_final(m*num_y+number,:)';1];
%         start_P_c(:,n)=[Points_c_final_undistortion((m-1)*num_y+1+number,:)';1];
%         end_P_c(:,n)=[Points_c_final_undistortion(m*num_y+number,:)';1];
%         n=n+1;
%         m=m+1;
%     end
% end
% 
% for i =1:1:size(l_P_c,1) %对法向量归一化
%      l_P_c(i,:)=l_P_c(i,:)/norm(l_P_c(i,1:2),2);
% end
% 
% % figure
% % for i =1:1:size(l_P_c,1)
% %      plot([1:2:1024],-([1:2:1024]*l_P_c(i,1)+l_P_c(i,3))/l_P_c(i,2))
% %      hold on
% % end
% 
% Tht_initial = zeros(6,1);
% 
% %find the closed point for camera line in reprojection line
% minfun = @(Tht)line2line(Tht,Ins_para_P_nodistortion,start_P_l,end_P_l,start_P_c,end_P_c,l_P_c);
% % Tht = fminsearch(minfun,Tht_initial,optimset('MaxFunEvals',40000,'MaxIter',40000,...
% %                          'Display','iter','Algorithm','levenberg-marquardt'));
% 
% options = optimoptions('lsqnonlin','StepTolerance',1e-8);
% Tht = lsqnonlin(minfun,Tht_initial,[],[],options);
% 
% T_P = Tht(4:6);
% R_P = rodrigues(Tht(1:3));
% RT_P = [R_P,T_P];

% 
% loss_withnoise_P_Method = Loss_of_P_Method(Tht,Ins_para_P_nodistortion,Points_c_final_undistortion,Points_l_final);
% 
% Points_c_final_undistortion_source=undistortion(Points_c_final_source,Ins_para_P,resolution); 
% 
% loss_withoutnoise_P_Method = Loss_of_P_Method(Tht,Ins_para_P_nodistortion,Points_c_final_undistortion_source,Points_l_final_source);
% 
% plot(image_points(1,:),image_points(2,:),'-*')
% axis equal;
% axis([0 resolution(1) 0 resolution(2)]);
% 







%% results
ranges = [3,5,10,20,30,50 100];

[projection_val_points,lidar_val_points]=gener_val_points(Ins_para,ranges,resolution,R_c2l,T_c2l);
projection_val_points_undistortion=undistortion(projection_val_points,Ins_para_P,resolution); 
projection_val_points_undistortion_insnoise=undistortion(projection_val_points,Ins_para_P_noise,resolution); 


for i =1:1:length(ranges)
    figure()
    subplot(1,3,1)
loss_P_Method(i) = Loss_of_P_Method(Tht,Ins_para_P_nodistortion,projection_val_points_undistortion,lidar_val_points{i},resolution);
grid on;

    subplot(1,3,2)
loss_Our_Method(i) = Loss_of_Our_Method(Calib_Re,projection_val_points,lidar_val_points{i},resolution);
grid on;
 
    subplot(1,3,3)
loss_P_Method_noised(i) = Loss_of_P_Method(Tht_withnoise,Ins_para_P_noise_nodistortion,projection_val_points_undistortion_insnoise,lidar_val_points{i},resolution);
grid on;

end

figure
plot(ranges,loss_P_Method,'-r');
hold on
plot(ranges,loss_Our_Method,'-k');
hold on
plot(ranges,loss_P_Method_noised,'--r');
close all
simresults = cell(1,3);

simresults{1}=loss_P_Method;
simresults{2}=loss_Our_Method;
simresults{3}=loss_P_Method_noised;

end




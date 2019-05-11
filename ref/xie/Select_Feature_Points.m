function [L_l,num,CornerPoint,CornerPoint_int,CornerImage]=Select_Feature_Points()

%% 选取特征边上的点，并进行标号

% file :txt
% 
% VER 1.0 谢诗超   2017/4/25

% close all ;
% clear all ;
% clc;


[filename,pathname]=uigetfile('*.txt','选择数据文件');

fid = fopen ([pathname filename]);


 points_flag = 0;
num_points = 10;
 i=1;
while 1
    
      
    tline = fgetl (fid);

   
 
    
    if (~points_flag)

        mat = regexp(tline,'POINTS','match');
        
        if (~isempty(mat))

            mat=regexp(tline,' ','split');
            num_points = str2num(mat{2});
            points_flag = 1;
            tline = fgetl (fid);
            loc = zeros(num_points,5);
        end
    



    else 
        

        mat=regexp(tline,' ','split');
        loc(i,1) = str2double(mat{1});
        loc(i,2) = str2double(mat{2});
        loc(i,3) = str2double(mat{3});
        loc(i,4) = str2double(mat{4});
        i = i +1;
    end
    if (i > num_points)
        break;
    end
end



loc(:,5)= (loc(:,1).^2+loc(:,2).^2+loc(:,3).^2).^0.5;
loc = loc';
fclose(fid);

%% 选点



% plot3(loc(1,:),loc(2,:),loc(3,:),'.','Markersize',0.5);
% xlabel ('x');
% ylabel ('y');
% zlabel ('z');
% h = waitbar(0,'等着吧');

selected_point = loc;


% c1=[-1,1,0];c2=[1,1,0];c3=[0,0.5,1];c4=[0,0.5,-1];
% 
% % R1 =[-1 ,  0 ,  0;
% %      0  , -1 , 0;
% %      0  ,  0 , 1;];
% R1=1;
% R2=1;
% % R2 = [1/2^0.5 , 1/2^0.5 , 0;
% %      1/2^0.5 , -1/2^0.5 ,  0;
% %      0       ,    0    ,  1;];
%  
%  i = 1;
% for n=1:1:num_points
% 
%     if c1*R1*R2*(selected_point(1:3,i)) < 0 
%         if c2*R1*R2*(selected_point(1:3,i)) < 0 
%             if c3*R1*R2*(selected_point(1:3,i)) < 0
%                 if c4*R1*R2*(selected_point(1:3,i)) < 0
%                     i=i+1;
%                 else
%                     selected_point(:,i)=[];
%                 end
%             else
%                     selected_point(:,i)=[];
%             end
%         else
%                     selected_point(:,i)=[];
%         end
%     else
%                     selected_point(:,i)=[];
%     end
%     
%   waitbar(n/num_points)
%                     
%     
% end
% close(h)

% selected_point_s=selected_point; %before trans

% selected_point(1:3,:)  = R1*R2*selected_point_s (1:3,:);

points_3d(1,:)=selected_point(2,:);
points_3d(2,:)=-selected_point(3,:);
points_3d(3,:)=-selected_point(1,:);
points_3d_int=selected_point(4,:);
points_3d_dis=selected_point(5,:);

backside = points_3d(3,:)<0.2;
points_3d(:,backside)=[];
points_3d_int(backside)=[];
points_3d_dis(backside)=[];
% points_3d(:,points_3d(3,:)>5.5)=[];
% points_3d(:,points_3d(3,:)<4)=[];
%% 投影过程

f=35e-3;
T = [f,0,0;0,f,0;0,0,1];
%投影
project_points = T*points_3d(1:3,:);

project_points(1,:) = project_points(1,:)./ project_points(3,:);
project_points(2,:) = project_points(2,:)./ project_points(3,:);
project_points(3,:) = project_points(3,:)./ project_points(3,:);

%平移坐标系
project_points(1,:) = project_points(1,:) + f;
project_points(2,:) = project_points(2,:) + f/2;

%投影到像素点
resolution = 1024;
size_u = 2*f/resolution;

image_points = (project_points(1:2,:)./size_u);
image_points = ceil(image_points);

image_dis = zeros(resolution/2,resolution);
image_int = zeros(resolution/2,resolution);
h = waitbar(0,'再等会');
sum_=0;

%记录指针矩阵
ptr = zeros (resolution/2,resolution);

% 投影到像素坐标
for u = 1:1:resolution 
    m=find (image_points(1,:)==u);
    for v = 1:1:resolution/2
      n=  find (image_points(2,m)==v);
      if (n)
      image_dis (v,u) = sum(points_3d_dis(m(n)))/length(n);
      image_int (v,u) = sum(points_3d_int(m(n)))/length(n);
      sum_=sum_+length(n);
      ptr(v,u)=m(n);
      end
    
    end
    
    waitbar(u/resolution)
end

close(h)

%% dis图像优化
 image_dis_s = image_dis ;
% image_dis = log(image_dis+1) ;
% image_dis = log(image_dis+1) ;
% image_dis = image_dis/max(max(image_dis));
% image_dis = imadjust(image_dis,[0.03,0.115],[0,1]);


%%
[CornerPoint,CornerPoint_int,CornerImage]=ShowTheCornerImg();
flag = 0;
dis = 1;

while (~flag)
  dis = dis+1;
 image_dis_s = image_dis ;
image_dis_s(image_dis_s<dis)=0;
image_dis_s(image_dis_s>dis+2)=0;













image_gin_  = image_dis_s/max(max(image_dis_s));
image_gin_(image_gin_>0.5)=1;
image_gin  = abs(image_gin_ -1 );

figure(2);
hold off;
imshow (image_gin)
hold on;
EagePointImage_out = EagePoint(image_gin_);
flag=input('clear?:');
end
 wid = 3;
 %X轴和Y轴到标定板边缘的距离
margin_x = 0.015;
margin_y = 0.008;   





for j =1 :1:2
    figure(2)
    [y,x] = ginput();
   
    x = int16(x);
    y = int16(y);
    for i = 1:1:length(x)
        if (EagePointImage_out(x(i),y(i)) ~=1)  %如果未选中端点
            [lx,ly]=find (EagePointImage_out(x(i)-wid:x(i)+wid,y(i)-wid:y(i)+wid)==1);   %找到wid近邻的端点位置
            x(i) = x(i)-wid-1+lx(1);
            y(i) = y(i)-wid-1+ly(1);
        end
    end
    plot (y,x,'bo')
    hold on
    
    for i = 1:1:length(x)
        L_l{j}(:,i) = points_3d (:,ptr(x(i),y(i)));
    end
end
CalibPatterFitting(T,f,size_u,L_l{1},L_l{2});

num=input('Please Input the number of lines:');
L_l = cell(1,num);

for j = 1:1:num
    
    figure(2)
    [y,x] = ginput();
   
    x = int16(x);
    y = int16(y);
    for i = 1:1:length(x)
        if (EagePointImage_out(x(i),y(i)) ~=1)  %如果未选中端点
            [lx,ly]=find (EagePointImage_out(x(i)-wid:x(i)+wid,y(i)-wid:y(i)+wid)==1);   %找到wid近邻的端点位置
            x(i) = x(i)-wid-1+lx(1);
            y(i) = y(i)-wid-1+ly(1);
        end
    end
    plot (y,x,'bo')
    hold on
    
    for i = 1:1:length(x)
        L_l{j}(:,i) = points_3d (:,ptr(x(i),y(i)));
    end
end

    
    
    
    
    
%     
%     
% [y1,x1] = ginput;
% wid = 2;
% x1 = int16(x1);
% y1 = int16(y1);
% for i = 1:1:length(x1)
%     if (image_gin(x1(i),y1(i)) >0.5)  %如果选中了白点
%         [lx1,ly1]=find (image_gin(x1(i)-wid:x1(i)+wid,y1(i)-wid:y1(i)+wid)<0.5);   %找到wid近邻的黑点位置
%         x1(i) = x1(i)-wid-1+lx1(1);
%         y1(i) = y1(i)-wid-1+ly1(1);
%     end
% end
% [y2,x2] = ginput;
% x2 = int16(x2);
% y2 = int16(y2);
% for i = 1:1:length(x2)
%     if (image_gin(x2(i),y2(i)) >0.5)
%         [lx2,ly2]=find (image_gin(x2(i)-wid:x2(i)+wid,y2(i)-wid:y2(i)+wid)<0.5);
%         x2(i) = x2(i)-wid-1+lx2(end);
%         y2(i) = y2(i)-wid-1+ly2(end);
%     end
% end
% 
% [y3,x3] = ginput;
% x3 = int16(x3);
% y3 = int16(y3);
% for i = 1:1:length(x3)
%     if (image_gin(x3(i),y3(i)) >0.5)
%         [lx3,ly3]=find (image_gin(x3(i)-wid:x3(i)+wid,y3(i)-wid:y3(i)+wid)<0.5);
%         x3(i) = x3(i)-wid-1+lx3(end);
%         y3(i) = y3(i)-wid-1+ly3(end);
%     end
% end
% 
% [y4,x4] = ginput;
% x4 = int16(x4);
% y4 = int16(y4);
% for i = 1:1:length(x4)
%     if (image_gin(x4(i),y4(i)) >0.5)
%         [lx4,ly4]=find (image_gin(x4(i)-wid:x4(i)+wid,y4(i)-wid:y4(i)+wid)<0.5);
%         x4(i) = x4(i)-wid-1+lx4(end);
%         y4(i) = y4(i)-wid-1+ly4(end);
%     end
% end
% 
% 
% 
% for i = 1:1:length(x1)
%     L1(:,i) = points_3d (:,ptr(x1(i),y1(i)));
% end
% 
% 
% 
% for i = 1:1:length(x2)
%     L2(:,i) = points_3d (:,ptr(x2(i),y2(i)));
% end
%     
% for i = 1:1:length(x3)
%     L3(:,i) = points_3d (:,ptr(x3(i),y3(i)));
% end
% 
% for i = 1:1:length(x4)
%     L4(:,i) = points_3d (:,ptr(x4(i),y4(i)));
% end

% global lx;  lx = L1;
% global ly;  ly = L2;



end
%% 选取特征边上的点，并进行标号

% file :txt
% 
% VER 1.0 谢诗超   2017/4/27

close all ;
clear all ;
clc;


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

%% 选点



% plot3(loc(:,1),loc(:,2),loc(:,3),'.','Markersize',0.5);
% xlabel ('x');
% ylabel ('y');
% zlabel ('z');
h = waitbar(0,'等着吧');

selected_point = loc;


c1=[-1,1,0];c2=[1,1,0];c3=[0,0.5,1];c4=[0,0.5,-1];

% R1 =[-1 ,  0 ,  0;
%      0  , -1 , 0;
%      0  ,  0 , 1;];
R1=1;
R2 = [1/2^0.5 , 1/2^0.5 , 0;
     1/2^0.5 , -1/2^0.5 ,  0;
     0       ,    0    ,  1;];
 
 i = 1;
for n=1:1:num_points

    if c1*R1*R2*(selected_point(1:3,i)) < 0 
        if c2*R1*R2*(selected_point(1:3,i)) < 0 
            if c3*R1*R2*(selected_point(1:3,i)) < 0
                if c4*R1*R2*(selected_point(1:3,i)) < 0
                    i=i+1;
                else
                    selected_point(:,i)=[];
                end
            else
                    selected_point(:,i)=[];
            end
        else
                    selected_point(:,i)=[];
        end
    else
                    selected_point(:,i)=[];
    end
    
  waitbar(n/num_points)
                    
    
end
close(h)

selected_point_s=selected_point; %before trans

selected_point(1:3,:)  = R1*R2*selected_point_s (1:3,:);

points_3d(1,:)=selected_point(1,:);
points_3d(2,:)=-selected_point(3,:);
points_3d(3,:)=-selected_point(2,:);
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
      image_dis (v,u) = sum(selected_point(5,m(n)))/length(n);
      image_int (v,u) = sum(selected_point(4,m(n)))/length(n);
      sum_=sum_+length(n);
      ptr(v,u)=m(n);
      end
    
    end
    
    waitbar(u/resolution)
end

close(h)

%% dis图像优化
image_dis_s = image_dis ;
image_dis = log(image_dis+1) ;
image_dis = log(image_dis+1) ;
image_dis = image_dis/max(max(image_dis));
image_dis = imadjust(image_dis,[0.03,0.115],[0,1]);

%% 选取两条边线
image_gin  = image_dis_s/max(max(image_dis_s));
image_gin  = abs(image_gin -1 );
imshow (image_gin)

[y1,x1] = ginput;

wid = 2;
x1 = int16(x1);
y1 = int16(y1);
for i = 1:1:length(x1)
    if (image_gin(x1(i),y1(i)) >0.5)
        [lx1,ly1]=find (image_gin(x1(i)-wid:x1(i)+wid,y1(i)-wid:y1(i)+wid)<0.5);
        x1(i) = x1(i)-wid-1+lx1(1);
        y1(i) = y1(i)-wid-1+ly1(1);
    end
end
[y2,x2] = ginput;
x2 = int16(x2);
y2 = int16(y2);
for i = 1:1:length(x2)
    if (image_gin(x2(i),y2(i)) >0.5)
        [lx2,ly2]=find (image_gin(x2(i)-wid:x2(i)+wid,y2(i)-wid:y2(i)+wid)<0.2);
        x2(i) = x2(i)-wid-1+lx2(end);
        y2(i) = y2(i)-wid-1+ly2(end);
    end
end


for i = 1:1:length(x1)
    L1(:,i) = points_3d (:,ptr(x1(i),y1(i)));
end



for i = 1:1:length(x2)
    L2(:,i) = points_3d (:,ptr(x2(i),y2(i)));
end
    
global lx;  lx = L1;
global ly;  ly = L2;


imshow (image_gin)
hold on
plot (y1,x1,'o')
hold on
plot (y2,x2,'o')
hold on

ddx = 0.15;ddy=0.15;

x = fmincon(@myfun,[-0.5,-0.5,2.5,1,0,0,0,1,0],[],[],[],[],[],[],@mycon);
dx = 0.158;
dy = 0.165;


X_ = x(4:6)';
Y_ = x(7:9)';
Z_ = cross ( x(4:6),x(7:9) )';
orig = x(1:3)'+dx*X_+dy*Y_;

%% 拟合标定板
global corner_point;
corner_point = cell (5,5);
corner_point{1,1} = orig;

for i = 1:1:5
    for j=1:1:5
        if (i==1 && j==1)
            continue;
        end
        corner_point{i,j} = corner_point{1,1} + (i-1)*ddx*X_+(j-1)*ddy*Y_;
    end
end
corner_project = cell (5,5);
m=1;
for i = 1:1:5
    for j=1:1:5
       
      
%投影
        corner_project{i,j} = T*corner_point{i,j};

        corner_project{i,j}(1) = corner_project{i,j}(1)./ corner_project{i,j}(3);
        corner_project{i,j}(2) = corner_project{i,j}(2)./ corner_project{i,j}(3);
        corner_project{i,j}(3) = corner_project{i,j}(3)./ corner_project{i,j}(3);

        %平移坐标系
        corner_project{i,j}(1) = corner_project{i,j}(1) + f;
        corner_project{i,j}(2) = corner_project{i,j}(2) + f/2;
        corner_project{i,j}(1:2) = corner_project{i,j}(1:2)./size_u;
        corner_x(m) =  corner_project{i,j}(1);
        corner_y(m) =  corner_project{i,j}(2);
        m=m+1;
    end
end
corner_point = flipud(corner_point);  %上下镜像，为了与图像识别结果对应


plot(corner_x,corner_y,'*')
hold on;







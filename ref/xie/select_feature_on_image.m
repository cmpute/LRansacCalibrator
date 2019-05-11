function [L,ori_im,f]=select_feature_on_image(num_L_c)
%% 
% clear;
%filename = 'Lena.jpg';
[filename,pathname]=uigetfile('*.jpg','选择数据文件');
X = imread([pathname filename]);     % 读取图像
% imshow(X);
Info = imfinfo([pathname filename]); %获取图像相关信息
if (Info.BitDepth > 8)
    f = rgb2gray(X);
end

%计算图像亮度f(x,y)在点(x,y)处的梯度-----------------------------------------------
% fx = [5 0 -5;8 0 -8;5 0 -5];          % 高斯函数一阶微分，x方向(用于改进的Harris角点提取算法)
ori_im = double(f) / 255;                   %unit8转化为64为双精度double64
% fx = [-2 -1 0 1 2];                     % x方向梯度算子(用于Harris角点提取算法)
% Ix = filter2(fx, ori_im);                % x方向滤波
% % fy = [5 8 5;0 0 0;-5 -8 -5];          % 高斯函数一阶微分，y方向(用于改进的Harris角点提取算法)
% fy = [-2; -1; 0; 1; 2];                     % y方向梯度算子(用于Harris角点提取算法)
% Iy = filter2(fy, ori_im);                % y方向滤波
% %构造自相关矩阵---------------------------------------------------------------
% Ix2 = Ix .^ 2;
% Iy2 = Iy .^ 2;
% Ixy = Ix .* Iy;
% clear Ix;
% clear Iy;
% h= fspecial('gaussian', [7 7], 2);        % 产生7*7的高斯窗函数，sigma=2
% Ix2 = filter2(h,Ix2);
% Iy2 = filter2(h,Iy2);
% Ixy = filter2(h,Ixy);
% %提取特征点---------------------------------------------------------------
height = size(ori_im, 1);
width = size(ori_im, 2);
% result = zeros(height, width);           % 纪录角点位置，角点处值为1
% R = zeros(height, width);
% Rmax = 0;                              % 图像中最大的R值
% k = 0.06; %k为常系数，经验取值范围为0.04~0.06
% for i = 1 : height
%     for j = 1 : width
%         M = [Ix2(i, j) Ixy(i, j); Ixy(i, j) Iy2(i, j)];             % auto correlation matrix
%         R(i,j) = det(M) - k * (trace(M)) ^ 2;                     % 计算R
%         if R(i,j) > Rmax
%             Rmax = R(i, j);
%         end;
%     end;
% end;
% %T = 0.01 * Rmax;%固定阈值，当R(i, j) > T时，则被判定为候选角点
% T = 0.1 * Rmax;%固定阈值，当R(i, j) > T时，则被判定为候选角点
% 
% %在计算完各点的值后，进行局部非极大值抑制-------------------------------------
% cnt = 0;
% for i = 2 : height-1
%     for j = 2 : width-1
%         % 进行非极大抑制，窗口大小3*3
%         if (R(i, j) > T && R(i, j) > R(i-1, j-1) && R(i, j) > R(i-1, j) && R(i, j) > R(i-1, j+1) && R(i, j) > R(i, j-1) && ...
%                 R(i, j) > R(i, j+1) && R(i, j) > R(i+1, j-1) && R(i, j) > R(i+1, j) && R(i, j) > R(i+1, j+1))
%             result(i, j) = 1;
%             cnt = cnt+1;
%         end;
%     end;
% end;
% i = 1;
%     for j = 1 : height
%         for k = 1 : width
%             if result(j, k) == 1;
%                 corners1(i, 1) = j;
%                 corners1(i, 2) = k;
%                 i = i + 1;
%             end; 
%         end;
%     end;
% [posc, posr] = find(result == 1);
figure,imshow(ori_im);
hold on;

[CornerPoint,boardSize]=detectCheckerboardPoints(f);
CornerPoint_int = ceil(CornerPoint);
CornerImage =  zeros(height, width);
CornerImage(CornerPoint_int(:,2)+(CornerPoint_int(:,1)-1)*height)=1;
plot(CornerPoint(:,1),CornerPoint(:,2),'r*')
hold on
% plot(posr, posc, 'r+');
% hold on;

%% 选取目标线上的点
x=cell(1,num_L_c);
y=cell(1,num_L_c);
% lx=cell(1,num_L);
% ly=cell(1,num_L);


  wid = 5;


for j =1:1:num_L_c
    
    [xx,yy] = ginput;   
  
    yy = int16(yy);
    xx=  int16(xx);
    for i = 1:1:length(yy)
        if (CornerImage(yy(i),xx(i)) ~=1)  %如果选中了非角点
            [lx,ly]=find (CornerImage(yy(i)-wid:yy(i)+wid,xx(i)-wid:xx(i)+wid)==1);   %找到wid近邻的角位置
            yy(i) = yy(i)-wid-1+lx(1);
            xx(i) = xx(i)-wid-1+ly(1);
        end
       mm = find (CornerPoint_int(:,1)==xx(i));
       nn = find (CornerPoint_int(mm,2)==yy(i));
       x{j}(i) = CornerPoint(mm(nn),1);
       y{j}(i) = CornerPoint(mm(nn),2);
    end
    plot (x{j},y{j},'o')
    hold on
    
end











% 
% for j =1:1:num_L_c
%     
%     [x{j},y{j}] = ginput;   
%     wid = 5;
%     y{j} = int16(y{j});
%     x{j}=  int16(x{j});
%     for i = 1:1:length(y{j})
%         if (result(y{j}(i),x{j}(i)) ~=1)  %如果选中了非角点
%             [lx,ly]=find (result(y{j}(i)-wid:y{j}(i)+wid,x{j}(i)-wid:x{j}(i)+wid)==1);   %找到wid近邻的角位置
%             y{j}(i) = y{j}(i)-wid-1+lx(1);
%             x{j}(i) = x{j}(i)-wid-1+ly(1);
%         end
%     end
%     plot (x{j},y{j},'o')
%     hold on
%     
% end


% 
% 
% 
% [x1,y1] = ginput;   
% wid = 5;
% y1 = int16(y1);
% x1 = int16(x1);
% for i = 1:1:length(y1)
%     if (result(y1(i),x1(i)) ~=1)  %如果选中了非角点
%         [lx1,ly1]=find (result(y1(i)-wid:y1(i)+wid,x1(i)-wid:x1(i)+wid)==1);   %找到wid近邻的角位置
%         y1(i) = y1(i)-wid-1+lx1(1);
%         x1(i) = x1(i)-wid-1+ly1(1);
%     end
% end
% plot (x1,y1,'o')
% hold on
% 
% 
% [x2,y2] = ginput;
% wid = 5;
% y2 = int16(y2);
% x2 = int16(x2);
% for i = 1:1:length(y2)
%     if (result(y2(i),x2(i)) ~=1)  %如果选中了非角点
%         [lx2,ly2]=find (result(y2(i)-wid:y2(i)+wid,x2(i)-wid:x2(i)+wid)==1);   %找到wid近邻的角位置
%         y2(i) = y2(i)-wid-1+lx2(1);
%         x2(i) = x2(i)-wid-1+ly2(1);
%     end
% end
% plot (x2,y2,'o')
% hold on
% 
% [x3,y3] = ginput;
% wid = 5;
% y3 = int16(y3);
% x3 = int16(x3);
% for i = 1:1:length(y3)
%     if (result(y3(i),x3(i)) ~=1)  %如果选中了非角点
%         [lx3,ly3]=find (result(y3(i)-wid:y3(i)+wid,x3(i)-wid:x3(i)+wid)==1);   %找到wid近邻的角位置
%         y3(i) = y3(i)-wid-1+lx3(1);
%         x3(i) = x3(i)-wid-1+ly3(1);
%     end
% end
% plot (x3,y3,'o')
% hold on
% 
% [x4,y4] = ginput;
% wid = 5;
% y4 = int16(y4);
% x4 = int16(x4);
% for i = 1:1:length(y4)
%     if (result(y4(i),x4(i)) ~=1)  %如果选中了非角点
%         [lx4,ly4]=find (result(y4(i)-wid:y4(i)+wid,x4(i)-wid:x4(i)+wid)==1);   %找到wid近邻的角位置
%         y4(i) = y4(i)-wid-1+lx4(1);
%         x4(i) = x4(i)-wid-1+ly4(1);
%     end
% end
% plot (x4,y4,'o')
% hold on

%% 用于拟合图片上选取的特征直线
L = zeros(num_L_c,2);
for i = 1:1:num_L_c
    x{i}=double(x{i});
    y{i}=double(y{i});
    L(i,:)= polyfit(x{i},y{i},1);
end

% 
% x1 = double(x1);
% x2 = double(x2);
% x3 = double(x3);
% x4 = double(x4);
% y1 = double(y1);
% y2 = double(y2);
% y3 = double(y3);
% y4 = double(y4);
% L(1,:) = polyfit(x1,y1,1);
% L(2,:) = polyfit(x2,y2,1);
% L(3,:) = polyfit(x3,y3,1);
% L(4,:) = polyfit(x4,y4,1);
for i=1:1:num_L_c
    fitting_line_u(i,:) = 1:1:width;
    fitting_line_v(i,:) = L(i,1)* fitting_line_u(i,:)+L(i,2);
end

for i =1:1:num_L_c
    plot(fitting_line_u(i,:),fitting_line_v(i,:));
    hold on 
end

end
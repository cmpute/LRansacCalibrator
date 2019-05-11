function [CornerPoint,CornerPoint_int,CornerImage]=ShowTheCornerImg()
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



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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


[CornerPoint,boardSize]=detectCheckerboardPoints(f);

 

figure()
hold off;
imshow(X);
hold on;


CornerPoint_int = ceil(CornerPoint);
CornerImage =  zeros(height, width);
CornerImage(CornerPoint_int(:,2)+(CornerPoint_int(:,1)-1)*height)=1;
plot(CornerPoint(:,1),CornerPoint(:,2),'r*','MarkerSize',2)
hold on
% plot(posr, posc, 'r+');
% hold on;




end





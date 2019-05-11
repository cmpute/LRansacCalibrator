function [CornerPoint,CornerPoint_int,CornerImage]=ShowTheCornerImg()
%% 
% clear;
%filename = 'Lena.jpg';
[filename,pathname]=uigetfile('*.jpg','ѡ�������ļ�');
X = imread([pathname filename]);     % ��ȡͼ��
% imshow(X);
Info = imfinfo([pathname filename]); %��ȡͼ�������Ϣ
if (Info.BitDepth > 8)
    f = rgb2gray(X);
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%����ͼ������f(x,y)�ڵ�(x,y)�����ݶ�-----------------------------------------------
% fx = [5 0 -5;8 0 -8;5 0 -5];          % ��˹����һ��΢�֣�x����(���ڸĽ���Harris�ǵ���ȡ�㷨)
ori_im = double(f) / 255;                   %unit8ת��Ϊ64Ϊ˫����double64
% fx = [-2 -1 0 1 2];                     % x�����ݶ�����(����Harris�ǵ���ȡ�㷨)
% Ix = filter2(fx, ori_im);                % x�����˲�
% % fy = [5 8 5;0 0 0;-5 -8 -5];          % ��˹����һ��΢�֣�y����(���ڸĽ���Harris�ǵ���ȡ�㷨)
% fy = [-2; -1; 0; 1; 2];                     % y�����ݶ�����(����Harris�ǵ���ȡ�㷨)
% Iy = filter2(fy, ori_im);                % y�����˲�
% %��������ؾ���---------------------------------------------------------------
% Ix2 = Ix .^ 2;
% Iy2 = Iy .^ 2;
% Ixy = Ix .* Iy;
% clear Ix;
% clear Iy;
% h= fspecial('gaussian', [7 7], 2);        % ����7*7�ĸ�˹��������sigma=2
% Ix2 = filter2(h,Ix2);
% Iy2 = filter2(h,Iy2);
% Ixy = filter2(h,Ixy);
% %��ȡ������---------------------------------------------------------------
height = size(ori_im, 1);
width = size(ori_im, 2);
% result = zeros(height, width);           % ��¼�ǵ�λ�ã��ǵ㴦ֵΪ1
% R = zeros(height, width);
% Rmax = 0;                              % ͼ��������Rֵ
% k = 0.06; %kΪ��ϵ��������ȡֵ��ΧΪ0.04~0.06
% for i = 1 : height
%     for j = 1 : width
%         M = [Ix2(i, j) Ixy(i, j); Ixy(i, j) Iy2(i, j)];             % auto correlation matrix
%         R(i,j) = det(M) - k * (trace(M)) ^ 2;                     % ����R
%         if R(i,j) > Rmax
%             Rmax = R(i, j);
%         end;
%     end;
% end;
% %T = 0.01 * Rmax;%�̶���ֵ����R(i, j) > Tʱ�����ж�Ϊ��ѡ�ǵ�
% T = 0.1 * Rmax;%�̶���ֵ����R(i, j) > Tʱ�����ж�Ϊ��ѡ�ǵ�
% 
% %�ڼ���������ֵ�󣬽��оֲ��Ǽ���ֵ����-------------------------------------
% cnt = 0;
% for i = 2 : height-1
%     for j = 2 : width-1
%         % ���зǼ������ƣ����ڴ�С3*3
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





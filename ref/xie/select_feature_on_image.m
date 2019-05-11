function [L,ori_im,f]=select_feature_on_image(num_L_c)
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

%% ѡȡĿ�����ϵĵ�
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
        if (CornerImage(yy(i),xx(i)) ~=1)  %���ѡ���˷ǽǵ�
            [lx,ly]=find (CornerImage(yy(i)-wid:yy(i)+wid,xx(i)-wid:xx(i)+wid)==1);   %�ҵ�wid���ڵĽ�λ��
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
%         if (result(y{j}(i),x{j}(i)) ~=1)  %���ѡ���˷ǽǵ�
%             [lx,ly]=find (result(y{j}(i)-wid:y{j}(i)+wid,x{j}(i)-wid:x{j}(i)+wid)==1);   %�ҵ�wid���ڵĽ�λ��
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
%     if (result(y1(i),x1(i)) ~=1)  %���ѡ���˷ǽǵ�
%         [lx1,ly1]=find (result(y1(i)-wid:y1(i)+wid,x1(i)-wid:x1(i)+wid)==1);   %�ҵ�wid���ڵĽ�λ��
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
%     if (result(y2(i),x2(i)) ~=1)  %���ѡ���˷ǽǵ�
%         [lx2,ly2]=find (result(y2(i)-wid:y2(i)+wid,x2(i)-wid:x2(i)+wid)==1);   %�ҵ�wid���ڵĽ�λ��
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
%     if (result(y3(i),x3(i)) ~=1)  %���ѡ���˷ǽǵ�
%         [lx3,ly3]=find (result(y3(i)-wid:y3(i)+wid,x3(i)-wid:x3(i)+wid)==1);   %�ҵ�wid���ڵĽ�λ��
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
%     if (result(y4(i),x4(i)) ~=1)  %���ѡ���˷ǽǵ�
%         [lx4,ly4]=find (result(y4(i)-wid:y4(i)+wid,x4(i)-wid:x4(i)+wid)==1);   %�ҵ�wid���ڵĽ�λ��
%         y4(i) = y4(i)-wid-1+lx4(1);
%         x4(i) = x4(i)-wid-1+ly4(1);
%     end
% end
% plot (x4,y4,'o')
% hold on

%% �������ͼƬ��ѡȡ������ֱ��
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
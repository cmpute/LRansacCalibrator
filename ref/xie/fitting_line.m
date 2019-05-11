line = lx';
scatter3(line(:,1), line(:,2), line(:,3),'filled')
hold on;

% 拟合的直线必过所有坐标的算数平均值
xyz01=mean(line,1);

% 协方差矩阵奇异变换，与拟合平面不同的是
% 所得直线的方向实际上与最大奇异值对应的奇异向量相同
centeredLine=bsxfun(@minus,line,xyz01);
[U,S,V]=svd(centeredLine);
direction1=V(:,1);

% % 画图
% t=-4:0.1:4;
% xx1=xyz01(1)+direction1(1)*t;
% yy1=xyz01(2)+direction1(2)*t;
% zz1=xyz01(3)+direction1(3)*t;
% plot3(xx1,yy1,zz1)

hold on 


line = ly';
scatter3(line(:,1), line(:,2), line(:,3),'filled')
hold on;

% 拟合的直线必过所有坐标的算数平均值
xyz02=mean(line,1);

% 协方差矩阵奇异变换，与拟合平面不同的是
% 所得直线的方向实际上与最大奇异值对应的奇异向量相同
centeredLine=bsxfun(@minus,line,xyz02);
[U,S,V]=svd(centeredLine);
direction2=V(:,1);

% % 画图
% t=-4:0.1:4;
% xx2=xyz02(1)+direction2(1)*t;
% yy2=xyz02(2)+direction2(2)*t;
% zz2=xyz02(3)+direction2(3)*t;
% plot3(xx2,yy2,zz2)

xlabel ('x');
ylabel ('y');
zlabel ('z');


k = 3;
hold on 

plot3 ([x(1),x(1)+k*x(4)],[x(2),x(2)+k*x(5)],[x(3),x(3)+k*x(6)],'-');
hold on 
plot3 ([x(1),x(1)+k*x(7)],[x(2),x(2)+k*x(8)],[x(3),x(3)+k*x(9)],'-');




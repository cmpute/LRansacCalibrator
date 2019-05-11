
%% 用于拟合图片上选取的特征直线

x1 = double(x1);
x2 = double(x2);
x3 = double(x3);
x4 = double(x4);
y1 = double(y1);
y2 = double(y2);
y3 = double(y3);
y4 = double(y4);
L(1,:) = polyfit(x1,y1,1);
L(2,:) = polyfit(x2,y2,1);
L(3,:) = polyfit(x3,y3,1);
L(4,:) = polyfit(x4,y4,1);
for i=1:1:4
    fitting_line_u(i,:) = 1:1:width;
    fitting_line_v(i,:) = L(i,1)* fitting_line_u(i,:)+L(i,2);
end

for i =1:1:4
    plot(fitting_line_u(i,:),fitting_line_v(i,:));
    hold on 
end




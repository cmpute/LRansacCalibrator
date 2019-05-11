function Points_c = SelectLinePoint(CornerPoint,CornerPoint_int,CornerImage,num_L_c)


%% 选取目标线上的点
x=zeros(1,num_L_c);
y=zeros(1,num_L_c);
% lx=cell(1,num_L);
% ly=cell(1,num_L);


  wid = 5;



    figure(1)
    [xx,yy] = ginput();   
  
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
       x(i) = CornerPoint(mm(nn),1);
       y(i) = CornerPoint(mm(nn),2);
    end
    plot (x,y,'o')
    hold on
    












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
Points_c = zeros(num_L_c,2);
for i = 1:1:num_L_c
   Points_c(i,:)= [x(i),y(i)];
end

% for i = 1:1:num_L_c
%     x{i}=double(x{i});
%     y{i}=double(y{i});
%     L(i,:)= polyfit(x{i},y{i},1);
% end

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
% for i=1:1:num_L_c
%     fitting_line_u(i,:) = 1:1:3000;
%     fitting_line_v(i,:) = L(i,1)* fitting_line_u(i,:)+L(i,2);
% end
% 
% for i =1:1:num_L_c
%     plot(fitting_line_u(i,:),fitting_line_v(i,:));
%     hold on 
% end

end
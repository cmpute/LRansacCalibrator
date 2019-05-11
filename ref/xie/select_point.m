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


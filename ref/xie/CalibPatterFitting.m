function []=CalibPatterFitting(T,f,size_u,lx,ly)
num_x=5;
num_y=5;

ddx = 0.15;ddy=0.15;

min_fun =@(X)myfun(X,lx,ly);

x = fmincon(min_fun,[-0.5,-0.5,2.5,1,0,0,0,1,0],[],[],[],[],[],[],@mycon);
dx = 0.158;
dy = 0.165;


X_ = x(4:6)';
Y_ = x(7:9)';
Z_ = cross ( x(4:6),x(7:9) )';
orig = x(1:3)'+dx*X_+dy*Y_;

%% 拟合标定板

corner_point = cell (num_x,num_y);
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


% plot(corner_x,corner_y,'r*')
hold on;   
for i =1:1:5
    plot(corner_x((i-1)*num_x+1:i*num_x),corner_y((i-1)*num_y+1:i*num_y),'b')
    hold on
    plot(corner_x(i:5:num_x*num_y),corner_y(i:5:num_x*num_y),'b')
    hold on
   
end
    




end
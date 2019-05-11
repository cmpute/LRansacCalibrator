function [Corner, Ptr_2,corner_point]=CalibPatterFitting_2(T,f,size_u,lx,ly,resolution)
num_x=6;
num_y=6;

ddx = 0.20;ddy=0.20;

min_fun =@(X)myfun(X,lx,ly);

x = fmincon(min_fun,[-0.5,-0.5,2.5,1,0,0,0,1,0],[],[],[],[],[],[],@mycon);
dx = 0.05;
dy = 0.05;


X_ = x(4:6)';
Y_ = x(7:9)';
Z_ = cross ( x(4:6),x(7:9) )';
orig = x(1:3)'+dx*X_+dy*Y_;

%% 拟合标定板

corner_point = cell (num_x,num_y);
corner_point{1,1} = orig;

for i = 1:1:num_x
    for j=1:1:num_y
        if (i==1 && j==1)
            continue;
        end
        corner_point{i,j} = corner_point{1,1} + (i-1)*ddx*X_+(j-1)*ddy*Y_;
    end
end
corner_project = cell (num_x,num_y);
m=1;
Ptr_2 = cell(resolution/2, resolution);
for i = 1:1:num_x
    for j=1:1:num_y
       
      
%投影
        corner_project{i,j} = T*corner_point{i,j};

        corner_project{i,j}(1) = corner_project{i,j}(1)./ corner_project{i,j}(3);
        corner_project{i,j}(2) = corner_project{i,j}(2)./ corner_project{i,j}(3);
        corner_project{i,j}(3) = corner_project{i,j}(3)./ corner_project{i,j}(3);

        %平移坐标系
        corner_project{i,j}(1) = corner_project{i,j}(1) + f;
        corner_project{i,j}(2) = corner_project{i,j}(2) + f/2;
        corner_project{i,j}(1:2) = corner_project{i,j}(1:2)./size_u;
        corner_project{i,j} = ceil(corner_project{i,j});
        Ptr_2{corner_project{i,j}(2),corner_project{i,j}(1)}=[i,j];
        corner_x(m) =  corner_project{i,j}(1);
        corner_y(m) =  corner_project{i,j}(2);
        m=m+1;
    end
end
% corner_point = flipud(corner_point);  %上下镜像，为了与图像识别结果对应
% CornerPoint_int = ceil([corner_x',corner_y']);
Corner =  zeros(resolution/2, resolution);
Corner(corner_y+(corner_x-1)*resolution/2)=1;


plot(corner_x,corner_y,'*')
hold on;


end
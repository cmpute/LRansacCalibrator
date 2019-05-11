

%% 求解最大似然 问题
%已知初始的H，和B。假设初始畸变参数为k1=0 k2=0,p1=0,p2=0,图像的光轴中心在图片的中央


Line = cell(1,3);

Line=edge(ori_im,'Roberts',0.1);
imshow(Line)


    
 min_fun1 =@(X)fun(X,L_3dpoints(1:6),Line{1});
 min_fun2 =@(X)fun(X,L_3dpoints(7:12),Line{2});
 min_fun3 =@(X)fun(X,L_3dpoints(13:17),Line{3});
 min_fun = @(X)(min_fun1(X)+min_fun2(X)+min_fun3(X));


Calib_Re = fminsearch(min_fun,[H;512;256;0;0;0;0],optimset('MaxFunEvals',10000,'MaxIter',10000,...
                        'Display','iter','Algorithm','levenberg-marquardt'));

 Calib_Re(11:14) = Calib_Re(11:14)/1e4;


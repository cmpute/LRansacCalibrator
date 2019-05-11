

flag = 1;
ii=1;
while (flag~=0)

[Points_l,num,CornerPoint,CornerPoint_int,CornerImage]=Select_Feature_Points_2();
Points_c = SelectLinePoint_2(CornerPoint,CornerPoint_int,CornerImage,num);
Points_l_final(ii:ii+num-1,:)=Points_l;
Points_c_final(ii:ii+num-1,:)=Points_c;
ii=ii+num;
flag =  input(['continue?';]);
end

[HH,H]=solv_the_initial_H_4(Points_c_final,Points_l_final);



figure
imshow(ori_im);
hold on;
vali_results(points_3d,HH,points_3d_dis,points_3d_int,1);



%% fine tune
initial_H = H(1:12);
%fine tune without H
initial_P = [resolution(1)/2;resolution(2)/2;0;0;0;0];
proportion = ones(6,1);

min_fun=@(P)FinetuneFunWithoutH(initial_H,P,Points_c_final,Points_l_final,resolution,1,proportion);
initial_P = fminsearch(min_fun,initial_P,optimset('MaxFunEvals',40000,'MaxIter',40000,...
                           'Display','iter', 'Algorithm','levenberg-marquardt','ToLX',1e-6));%'Display','iter',
                       
%对每个变量进行近似归一化                       
proportion=initial_P;
min_fun=@(P)FinetuneFunWithoutH(initial_H,P,Points_c_final,Points_l_final,resolution,1,proportion);
initial_P = fminsearch(min_fun,ones(6,1),optimset('MaxFunEvals',40000,'MaxIter',40000,...
                            'Algorithm','levenberg-marquardt','ToLX',1e-6));%'Display','iter',
% P=initial_P.*proportion;


proportion_ =[initial_H;proportion.*initial_P];
min_fun = @(P) FinetuneFun(P,Points_c_final,Points_l_final,resolution,1,proportion_);
loss_min = 10000;
rand_value = ones(18,1);
for i=1:1:30
   
    Calib_Re = fminsearch(min_fun,rand_value,optimset('MaxFunEvals',40000,'MaxIter',40000,...
                            'Algorithm','levenberg-marquardt','ToLX',1e-6));%'Display','iter',
    
                        
%       loss = Loss_of_Our_Method(Calib_Re,Points_c_final,Points_l_final,resolution,0);
                
                        
                        
      loss = FinetuneFun(Calib_Re,Points_c_final,Points_l_final,resolution,1,proportion_);
    if loss<loss_min
        loss_min=loss;
        Calib_Re_final = Calib_Re;
    end
     
     rand_value = (0.9+0.2*rand(18,1));  
     i            
end
Calib_Re = Calib_Re_final.*proportion_;
% Calib_Re = fminsearch(min_fun,[H(1:12);512;256;0;0;0;0],optimset('MaxFunEvals',40000,'MaxIter',40000,...
%                         'Display','iter','Algorithm','levenberg-marquardt'));


%average loss of pixel



points_3d_image=vali_results_fine(points_3d,Calib_Re,points_3d_dis,points_3d_int,resolution,1);


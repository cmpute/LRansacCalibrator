

flag = 1;
ii=1;
while (flag~=0)

[L_l,num,CornerPoint,CornerPoint_int,CornerImage]=Select_Feature_Points_2();
L_c = SelectLinePoint(CornerPoint,CornerPoint_int,CornerImage,num);
L_l_final(ii:ii+num-1)=L_l;
L_c_final(ii:ii+num-1,:)=L_c;
ii=ii+num;
flag =  input(['continue?';]);
end

[HH,B,H, L_3dpoints]=solv_the_initial_H_3(L_c_final,L_l_final);

figure
imshow(ori_im);
hold on;
vali_results(points_3d,HH,B);



%%
vali_results(Points_l_final([26,50],:)',HH,B);
vali_results(L_l_final{2},HH,B);

vali_results_fine(points_3d,Calib_Re);

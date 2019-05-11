function Points_c_final_unditortion=undistortion(Points_c_final,Ins_para,resolution)
    
    u0= Ins_para(3);
    v0= Ins_para(4);
    u_x = zeros(1,size(Points_c_final,1));
    u_y=u_x;
    h = waitbar(0,'wait');
for i =1:1:size(Points_c_final,1)
     u_x_=Points_c_final(i,1)-u0;  %转到相机坐标
     u_y_=Points_c_final(i,2)-v0;  
    fmin = @(P)fmin_undistortion (P,u_x_,u_y_,Ins_para);
    U=fmincon(fmin,[u_x_,u_y_],[],[],[],[],[-resolution(1)/2,-resolution(2)/2],[resolution(1)/2,resolution(2)/2],[],optimset('MaxFunEvals',10000,'MaxIter',10000,...
                            'TolFun',0.001,'TolX',0.001));
     u_x(i)=U(1);
     u_y(i)=U(2);
     waitbar(i/size(Points_c_final,1))
     
end  
    Points_c_final_unditortion(:,1)=u_x'+u0;
    Points_c_final_unditortion(:,2)=u_y'+v0;

   close(h);


end
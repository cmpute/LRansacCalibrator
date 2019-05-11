function f =fun (X,L_3dpoints,Line)
 resolution = 1024;
X(11:14) = X(11:14)/1e4;

f=0;
[x, y] = meshgrid(1:size(Line,2), 1:size(Line,1));
for i = 1:1:length(L_3dpoints)
    for j = 1:1:size(L_3dpoints{i},2)    %a=L1   b=-1   c=L2
%          AA(rr,:)=[L(i,1) , -1,L(i,2)] ;     
         u_v=  (reshape(X(1:6),3,2))'*L_3dpoints{i}(:,j)+reshape(X(7:8),2,1); %直接投影
         u_x=u_v(1)-X(9);  %转到相机坐标
         u_y=u_v(2)-X(10);  
        %考虑畸变
        r_2 = u_x^2+u_y^2;
        u_x_ = u_x*(1+X(11)*r_2+X(12)*r_2^2)+(2*X(13)*u_x*u_y+X(14)*(r_2+2*u_x^2));
        u_y_ = u_y*(1+X(11)*r_2+X(12)*r_2^2)+(2*X(14)*u_x*u_y+X(13)*(r_2+2*u_y^2));
        
        u_x = u_x_ +X(9);   %转回像素坐标
        u_y = u_y_ +X(10);
        Mask_Line = 0;
        r0=0.1;
        while (~Mask_Line)
            if (r0>=100)
                break;
            end
       
         Mask = (x-u_x).^2+(y-u_y).^2-r0^2<=0;
         Mask_Line = Mask & Line;
         r0=r0+0.1;
        end
        f = f+r0;
%         f_err(rr)=AA(rr,:)*[u_x;u_y;1];
%         rr= rr+1;
    end
end
% f = norm(f_err);


end
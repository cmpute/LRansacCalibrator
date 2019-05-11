function loss = InitialTuneFun(H,Points_c_final,Points_l_final)
 num = size(Points_c_final,1);
 HH = [H(1:4)';H(5:8)';H(9:12)'];
 

 
 ProjectPoints = HH *[Points_l_final';ones(1,size(Points_l_final',2))];
 ProjectPoints(1,:)=ProjectPoints(1,:)./ProjectPoints(3,:);
 ProjectPoints(2,:)=ProjectPoints(2,:)./ProjectPoints(3,:);
 ProjectPoints(3,:)=ProjectPoints(3,:)./ProjectPoints(3,:);
 

        err = Points_c_final' - [ProjectPoints(1,:);ProjectPoints(2,:)];
       
        loss =0;
        for i = 1:1:num
            loss = loss+norm(err(:,i),2)^2;
        end
     loss = (loss/num)^0.5;

end
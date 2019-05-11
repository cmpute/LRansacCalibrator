function Tht=P_method_cal_reality(Points_c_final_undistortion,Points_l_final,num_x,num_y,num_cal,Ins_para_P_nodistortion)



l_P_c = zeros((num_x+num_y)*num_cal,3);
start_P_l = zeros(4,(num_x+num_y)*num_cal);
end_P_l = zeros(4,(num_x+num_y)*num_cal);
start_P_c = zeros(3,(num_x+num_y)*num_cal);
end_P_c = zeros(3,(num_x+num_y)*num_cal);
n=1;
for j =1:1:num_cal
        number =(j-1)*num_x*num_y;
        m=1;
    for i =1+number:1:num_y+number
        l_P_c(n,:)=(cross([Points_c_final_undistortion(i,:)';1],[Points_c_final_undistortion(i+(num_x-1)*num_y,:)';1]))';
        start_P_l(:,n)=[Points_l_final(i,:)';1];
        end_P_l(:,n)=[Points_l_final(i+(num_x-1)*num_y,:)';1];
        start_P_c(:,n)=[Points_c_final_undistortion(i,:)';1];
        end_P_c(:,n)=[Points_c_final_undistortion(i+(num_x-1)*num_y,:)';1];
        n=n+1;
    end
    for i =1+number:1:num_x+number
        l_P_c(n,:)=(cross([Points_c_final_undistortion((m-1)*num_y+1+number,:)';1],[Points_c_final_undistortion(m*num_y+number,:)';1]))';
        start_P_l(:,n)=[Points_l_final((m-1)*num_y+1+number,:)';1];
        end_P_l(:,n)=[Points_l_final(m*num_y+number,:)';1];
        start_P_c(:,n)=[Points_c_final_undistortion((m-1)*num_y+1+number,:)';1];
        end_P_c(:,n)=[Points_c_final_undistortion(m*num_y+number,:)';1];
        n=n+1;
        m=m+1;
    end
end

for i =1:1:size(l_P_c,1) %对法向量归一化
     l_P_c(i,:)=l_P_c(i,:)/norm(l_P_c(i,1:2),2);
end

% figure
for i =13:1:24
     plot([1:2:1920],-([1:2:1920]*l_P_c(i,1)+l_P_c(i,3))/l_P_c(i,2))
     hold on
end



Tht_initial = zeros(6,1);

%find the closed point for camera line in reprojection line
minfun = @(Tht)line2line(Tht,Ins_para_P_nodistortion,start_P_l,end_P_l,start_P_c,end_P_c,l_P_c);
% Tht = fminsearch(minfun,Tht_initial,optimset('MaxFunEvals',40000,'MaxIter',40000,...
%                          'Display','iter','Algorithm','levenberg-marquardt'));

options = optimoptions('lsqnonlin','StepTolerance',1e-8);
Tht = lsqnonlin(minfun,Tht_initial,[],[],options);

T_P = Tht(4:6);
R_P = rodrigues(Tht(1:3));
RT_P = [R_P,T_P];


end
function loss = line2line(Tht,Ins_para_P_nodistortion,start_P_l,end_P_l,start_P_c,end_P_c,l_P_c)
R = rodrigues(Tht(1:3));
T = Tht(4:6);
RT_P = [R,T];
%  RT_P =[0.8028   -0.0786    0.5910   -0.3147
%     0.5251    0.5626   -0.6385   -0.4058;
%    -0.2824    0.8230    0.4929    0.3730;];
[start_P_l_project,~] = project(RT_P*start_P_l,Ins_para_P_nodistortion);
[end_P_l_project,~] = project(RT_P*end_P_l,Ins_para_P_nodistortion);
num = size(start_P_l_project,2);
l_P_l_project=zeros(num,3);
start_P_l_project_verti=zeros(num,3);
end_P_l_project_verti=zeros(num,3);

start_point = zeros(3,num);
end_point = zeros(3,num);

for i = 1:1:num
    l_P_l_project(i,:) = cross(start_P_l_project(:,i),end_P_l_project(:,i))';
    start_P_l_project_verti(i,:)=[l_P_l_project(i,2)/l_P_l_project(i,1),-1,...
                                                 -l_P_l_project(i,2)/l_P_l_project(i,1)*start_P_c(1,i)+start_P_c(2,i) ];
    end_P_l_project_verti(i,:)=[l_P_l_project(i,2)/l_P_l_project(i,1),-1,...
                                                 -l_P_l_project(i,2)/l_P_l_project(i,1)*end_P_c(1,i)+end_P_c(2,i) ];
end



for i =1:1:num
    A_start = [l_P_c(i,1),l_P_c(i,2);start_P_l_project_verti(i,1),start_P_l_project_verti(i,2)];
    B_start = -[l_P_c(i,3);start_P_l_project_verti(i,3)];
    A_end = [l_P_c(i,1),l_P_c(i,2);end_P_l_project_verti(i,1),end_P_l_project_verti(i,2)];
    B_end = -[l_P_c(i,3);end_P_l_project_verti(i,3)];
    
    start_solutionX = A_start\ B_start;
    end_solutionX = A_end\B_end;
    
    if (start_solutionX-start_P_l_project(1:2,i))'*(end_P_l_project(1:2,i)-start_P_l_project(1:2,i))<0
        start_point(:,i)=start_P_l_project(:,i);
    elseif norm(start_solutionX-start_P_l_project(1:2,i),2)>norm(end_P_l_project(1:2,i)-start_P_l_project(1:2,i),2)
         start_point(:,i)=end_P_l_project(:,i);
    else
        start_point(:,i)=[start_solutionX;1];
    end
        
    if (end_solutionX-start_P_l_project(1:2,i))'*(end_P_l_project(1:2,i)-start_P_l_project(1:2,i))<0
        end_point(:,i)=start_P_l_project(:,i);
    elseif norm(end_solutionX-start_P_l_project(1:2,i),2)>norm(end_P_l_project(1:2,i)-start_P_l_project(1:2,i),2)
         end_point(:,i)=end_P_l_project(:,i);
    else
        end_point(:,i)=[end_solutionX;1] ;
    end
end
    
% loss =0;
loss = zeros(2*num,1);
j=1;
for i =1:1:num
%     loss = loss+abs(l_P_c(i,:)*end_point(:,i));
%     loss = loss+abs(l_P_c(i,:)*start_point(:,i));
    loss(j)=l_P_c(i,:)*end_point(:,i);
    j=j+1;
    loss(j)=l_P_c(i,:)*start_point(:,i);
    j=j+1;
end
    loss =loss/2/num;
end
    
    
    
    
    
    
    
    


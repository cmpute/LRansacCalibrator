
function loss = Loss_of_P_Method(Tht,Ins_para_P_nodistortion,Points_c_final_undistortion,Points_l_final,resolution)
 num = size(Points_c_final_undistortion,1);
 R = rodrigues(Tht(1:3));
 T = Tht(4:6);
 RT_P = [R,T];
[Points_project,~] = project(RT_P*[Points_l_final';ones(1,num)],Ins_para_P_nodistortion);
 
loss_matrix = Points_project(1:2,:)-Points_c_final_undistortion';
 loss=0;
for i=1:1:num
    loss = loss+norm(loss_matrix(:,i),2)^2;
    
end
loss = (loss/num)^0.5;


% plot(Points_project(1,:),Points_project(2,:),'*')
% hold on
% plot(Points_c_final_undistortion(:,1),Points_c_final_undistortion(:,2),'o')
% hold on
% 
% 
% axis equal;
% axis([0 resolution(1) 0 resolution(2)]);




end
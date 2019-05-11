function [HH,B,H, L_3dpoints]=solv_the_initial_H_3(L_c_final,L_l_final)

% global L_3dpoints;
L_3dpoints = cell(1,length(L_l_final));
for i = 1:1:length(L_l_final)
    L_3dpoints{i} = L_l_final{i};
end
rr = 1;
for i = 1:1:length(L_3dpoints)
    for j = 1:1:size(L_3dpoints{i},2)    %a=L1   b=-1   c=L2
        A(rr,:)=[L_c_final(i,1)*(L_3dpoints{i}(:,j)') , -L_3dpoints{i}(:,j)',L_c_final(i,1),-1 ] ;     
        b(rr,:)=-L_c_final(i,2);
        rr= rr+1;
    end
end
[rr,~] = size(A); %记录共有多少个点
[U,S,V]=svd(A);
%%%
  SS = S(1:8,1:8)^-1;
 SS(:,9:rr)=zeros(8,rr-8);
  H=V*SS*U'*b;
   %% 去除误差大的点
%   err = abs(A*H-b)./b;
%   A(err>0.03,:)=[];
%   b(err>0.03,:)=[];
%   [rr,~] = size(A); %记录共有多少个点
%   [U,S,V]=svd(A);
%   SS = S(1:8,1:8)^-1;
%   SS(:,9:rr)=zeros(8,rr-8);
%   H=V*SS*U'*b;
  %%
% [~,bb] = size(A);
% lamda = max(max(S))^2/600;
% H = (A'*A+lamda*eye(bb))\A'*b;

HH =  (reshape(H(1:6),3,2))';
 B  =  reshape(H(7:8),2,1);

%%

for i = 1:1:length(L_3dpoints)
    for j = 1:1:size(L_3dpoints{i},2)   
        trans_project{i}(:,j)=HH*[L_3dpoints{i}(:,j)]+B;
       
    end
end  

%%


% for i = 1:1:length(trans_project)
%     
%     plot (trans_project{i}(1,:),trans_project{i}(2,:),'*')
%     hold on; 
% end  
end
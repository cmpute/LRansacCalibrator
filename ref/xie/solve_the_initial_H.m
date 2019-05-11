global A;
% global L_3dpoints;
L_3dpoints{1} = L1;
L_3dpoints{2} = L2;
L_3dpoints{3} = L3;
% L_3dpoints{4} = L4;

rr = 1;
for i = 1:1:length(L_3dpoints)
    for j = 1:1:size(L_3dpoints{i},2)    %a=L1   b=-1   c=L2
        A(rr,:)=[L(i,1)*L_3dpoints{i}(:,j)',L(i,1) , -L_3dpoints{i}(:,j)',-1, L(i,2)*L_3dpoints{i}(:,j)',L(i,2)] ;   
        rr= rr+1;
    end
end

%%
H = fmincon(@funf,[0,0,0,0,0,0,0,0,0,0,1,0],[],[],[],[],[],[],@conf);
HH = (reshape(H,4,3))';
%%

for i = 1:1:length(L_3dpoints)
    for j = 1:1:size(L_3dpoints{i},2)   
        trans_project{i}(:,j)=HH*[L_3dpoints{i}(:,j);1];
        trans_project{i}(:,j)=trans_project{i}(:,j)./trans_project{i}(3,j);
    end
end  

%%


for i = 1:1:length(trans_project)
    
    plot (trans_project{i}(1,:),trans_project{i}(2,:),'*')
    hold on; 
end  








  
        
        
        
        
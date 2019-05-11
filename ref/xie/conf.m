%% Ö±Ïß 
function [c,ceq]= conf(H)
%     global A;
%     global L_3dpoints;
%     HH = (reshape(H,4,3))';
    ceq= H(9)^2+ H(10)^2+ H(11)^2 -1; 
%    ii = 1;
%     for i = 1:1:length(L_3dpoints)
%         for j = 1:1:size(L_3dpoints{i},2)   
%             trans_project{i}(:,j)=HH*[L_3dpoints{i}(:,j);1];
%             trans_project{i}(:,j)=trans_project{i}(:,j)./trans_project{i}(3,j);
%             c(ii)=-trans_project{i}(1,j);
%             c(ii+1)=-trans_project{i}(2,j);
%             ii=ii+2;
%         end
%     end     
 c = [];
end
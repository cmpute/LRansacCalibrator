function [HH,H]=solv_the_initial_H_4(Points_c_final,Points_l_final)


num_sample  = size(Points_c_final,1);%%样本数
A = zeros(3*num_sample ,12+num_sample );
% b1 = zeros(size(Points_c_final,1),1);
% b2 = zeros(size(Points_c_final,1),1);
j = 1;

for i = 1:1:size(Points_c_final,1)
   
        A(j,1:12)=[Points_l_final(i,:),1,zeros(1,8)];
        A(j+1,1:12)=[zeros(1,4),Points_l_final(i,:),1,zeros(1,4)];
        A(j+2,1:12)=[zeros(1,8),Points_l_final(i,:),1];
        j=j+3;
        
        
        A((i-1)*3+1:(i-1)*3+3,12+i)  =  -[Points_c_final(i,1);Points_c_final(i,2);1];
        
        
        
%         
%         b1(i)=Points_c_final(i,1);
%         b2(i)=Points_c_final(i,2);
       
    
end
%令最后一个未知数为1，解非齐次最小二乘问题
% b = -A(:,end);
% A(:,end) = [];
% 
% [rr,~] = size(A); 
% [U,S,V]=svd(A);
% 
% num_var = 12+num_sample-1;
% 
% SS = S(1:num_var,1:num_var)^-1;
% SS(:,num_var+1:rr)=zeros(num_var,rr-num_var);
% H = V*SS*U'*b;

b = -A(:,end);
A(:,end) = [];

[rr,cc] = size(A); 
[U,S,V]=svd(A);

num_var = cc;

SS = S(1:num_var,1:num_var)^-1;
SS(:,num_var+1:rr)=zeros(num_var,rr-num_var);
H = V*SS*U'*b;







% H1=V*SS*U'*b1;
% H2=V*SS*U'*b2;

%   err = ((abs(A*H1-b1)./b1).^2+(abs(A*H2-b2)./b2).^2).^0.5;
%   A(err>0.05,:)=[];
%   b1(err>0.05,:)=[];
%   b2(err>0.05,:)=[];
%   [rr,~] = size(A); %记录共有多少个点
%   [U,S,V]=svd(A);
%   SS = S(1:4,1:4)^-1;
% SS(:,5:rr)=zeros(4,rr-4);
% H1=V*SS*U'*b1;
% H2=V*SS*U'*b2;


% 
%   [~,bb] = size(A);
%   lamda = max(max(S))^2/10000;
%   H1 = (A'*A+lamda*eye(bb))\A'*b1;
%   H2 = (A'*A+lamda*eye(bb))\A'*b2;



% HH = [H1(1:3)';H2(1:3)'];
% B = [H1(4);H2(4)];




% HH =  (reshape(H(1:6),3,2))';
% B  =  reshape(H(7:8),2,1);

%%
min_fun = @(PH) InitialTuneFun(PH,Points_c_final,Points_l_final);
H(13:end)=[];

H = fminsearch(min_fun,H,optimset('MaxFunEvals',40000,'MaxIter',40000,...
                        'Algorithm','levenberg-marquardt'));


HH = [H(1:4)';H(5:8)';H(9:12)'];



%%


% for i = 1:1:length(trans_project)
%     
%     plot (trans_project{i}(1,:),trans_project{i}(2,:),'*')
%     hold on; 
% end  
end
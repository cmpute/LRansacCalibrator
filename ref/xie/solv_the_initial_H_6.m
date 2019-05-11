function [HH,H]=solv_the_initial_H_6(Points_c_final,Points_l_final)
%解齐次问题

num_sample  = size(Points_c_final,1);%%样本数
A = zeros(2*num_sample ,12 );
% b1 = zeros(size(Points_c_final,1),1);
% b2 = zeros(size(Points_c_final,1),1);
j = 1;

for i = 1:1:size(Points_c_final,1)
   
        A(j,1:12)=[[Points_l_final(i,:),1],zeros(1,4),-Points_c_final(i,1)*[Points_l_final(i,:),1]];
        A(j+1,1:12)=[zeros(1,4),[Points_l_final(i,:),1],-Points_c_final(i,2)*[Points_l_final(i,:),1]];
        
        j=j+2;
        
end




b = -A(:,end);
A(:,end) = [];

[rr,cc] = size(A); 
[U,S,V]=svd(A);

num_var = cc;

SS = S(1:num_var,1:num_var)^-1;
SS(:,num_var+1:rr)=zeros(num_var,rr-num_var);
H = V*SS*U'*b;


HH = [H(1:4)';H(5:8)';H(9:11)',1];




end
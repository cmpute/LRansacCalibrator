function [HH,H]=solv_the_initial_H_7(Points_c_final,Points_l_final)


num_sample  = size(Points_c_final,1);%%Ñù±¾Êý
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
[U,S,V]=svd(A);


H = V(1:12,end);



HH = [H(1:4)';H(5:8)';H(9:12)'];


% HH = [H1(1:3)';H2(1:3)'];
% B = [H1(4);H2(4)];




% HH =  (reshape(H(1:6),3,2))';
% B  =  reshape(H(7:8),2,1);

%%


%%


% for i = 1:1:length(trans_project)
%     
%     plot (trans_project{i}(1,:),trans_project{i}(2,:),'*')
%     hold on; 
% end  
end
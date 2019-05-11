
function loss = FinetuneFun(P,Points_c_final,Points_l_final,resolution,mode,proportion_)
 num = size(Points_c_final,1);
 if nargin==6
 P=P.*proportion_;
 end
 
 
 HH = [P(1:4)';P(5:8)';P(9:12)'];
 
 u0 = P(13);
 v0 = P(14);
 
 k1 = P(15);
 k2 = P(16);
 
 p1 = P(17);
 p2 = P(18);
 
 ProjectPoints = HH *[Points_l_final';ones(1,size(Points_l_final',2))];
 ProjectPoints(1,:)=ProjectPoints(1,:)./ProjectPoints(3,:);
 ProjectPoints(2,:)=ProjectPoints(2,:)./ProjectPoints(3,:);
 ProjectPoints(3,:)=ProjectPoints(3,:)./ProjectPoints(3,:);
 
 u_x=ProjectPoints(1,:)-u0;  %ת���������
 u_y=ProjectPoints(2,:)-v0;  
        %���ǻ���
        r_2 = u_x.^2+u_y.^2;
        u_x_ = u_x.*(1+k1*r_2+k2*r_2.^2)+(2*p1*u_x.*u_y+p2*(r_2+2*u_x.^2));
        u_y_ = u_y.*(1+k1*r_2+k2*r_2.^2)+(2*p2*u_x.*u_y+p1*(r_2+2*u_y.^2));
        
        u_x = u_x_ +u0;   %ת����������
        u_y = u_y_ +v0; 
        
        err = Points_c_final' - [u_x;u_y];
        lmd = 0.0001;
        loss =0;
        for i = 1:1:num
            loss = loss+norm(err(:,i),2)^2;
        end
        if mode==1
            loss =loss/num+lmd*((u0-resolution(1)/2)^2+(v0-resolution(2)/2)^2);
        
        else 
            loss = (loss/num)^0.5;
        end

%         loss = (norm(err(1,:),2)^2 + norm(err(2,:),2)^2)/length(u_x)+lmd*((u0-resolution(1)/2)^2+(v0-resolution(2)/2)^2);


end
function []=evaluate_calibration_result(Points_c_final,Points_c_final_undistortion,Points_l_final)
%% evaluate calibration result

%% 
 HH = [Calib_Re(1:4)';Calib_Re(5:8)';Calib_Re(9:12)'];
 
 u0 = Calib_Re(13);
 v0 = Calib_Re(14);
 
 k1 = Calib_Re(15);
 k2 = Calib_Re(16);
 
 p1 = Calib_Re(17);
 p2 = Calib_Re(18);
 
 ProjectPoints = HH *[Points_l_final';ones(1,size(Points_l_final',2))];
 ProjectPoints(1,:)=ProjectPoints(1,:)./ProjectPoints(3,:);
 ProjectPoints(2,:)=ProjectPoints(2,:)./ProjectPoints(3,:);
 ProjectPoints(3,:)=ProjectPoints(3,:)./ProjectPoints(3,:);
 

  
  u_x=ProjectPoints(1,:)-u0;  %转到相机坐标
  u_y=ProjectPoints(2,:)-v0;  
 

        %考虑畸变
        r_2 = u_x.^2+u_y.^2;
        u_x_ = u_x.*(1+k1*r_2+k2*r_2.^2)+(2*p1*u_x.*u_y+p2*(r_2+2*u_x.^2));
        u_y_ = u_y.*(1+k1*r_2+k2*r_2.^2)+(2*p2*u_x.*u_y+p1*(r_2+2*u_y.^2));
        
        u_x = u_x_ +u0;   %转回像素坐标
        u_y = u_y_ +v0;
        Loss_our = zeros(num_x*num_y,num_cal);
        Loss_P = zeros(num_x*num_y,num_cal);
        
%         for i = 1:1:10
%             figure()
%             plot(Points_c_final((i-1)*36+1:i*36,1),Points_c_final((i-1)*36+1:i*36,2),'ro','MarkerSize',5);
%             hold on
%             plot(u_x((i-1)*36+1:i*36),u_y((i-1)*36+1:i*36),'w*','MarkerSize',5);
%             axis([0 resolution(1) 0 resolution(2)]);
%             axis equal
%               
%         end
        
        m=1;
        for i =1:1:num_cal
            for j =1:1:num_x*num_y
                Loss_our(j,i) = ((Points_c_final(m,1)-u_x(m))^2+(Points_c_final(m,2)-u_y(m))^2)^0.5;
                Loss_P(j,i)   =  Loss_of_P_Method(Tht,Ins_para_P_nodistortion,Points_c_final_undistortion(m,:),Points_l_final(m,:),resolution);
                m=m+1;
            end
            
        end
        
        
            mean_our = sum(sum(Loss_our))/(num_cal*num_x*num_y);
            mean_P= sum(sum(Loss_P))/(num_cal*num_x*num_y);
            Loss_our_ = reshape(Loss_our,1,num_cal*num_x*num_y);
            Loss_P_ = reshape(Loss_P,1,num_cal*num_x*num_y);
            
            outLiers_our = Loss_our_>2*mean_our;
            outLiers_P = Loss_P_>2*mean_P;
            
            Loss_our_(outLiers_our)=[];
            Loss_P_(outLiers_P)=[];
        
            RMS_our = (sum(Loss_our_.^2)/length(Loss_our_))^0.5;
            RMS_P = (sum(Loss_P_.^2)/length(Loss_P_))^0.5;
        
        

 %% plot
posit = [];

 plotErr=[];
gcolor=[];

 m=1;
 for i =1:1:num_cal
     plotErr = [plotErr,Loss_P(:,i),Loss_our(:,i)];
     posit = [ posit,m,m+1];
gcolor=[gcolor,1,5];
     m=m+3;

 end
 
 
 figure
 hold on
 boxplot(plotErr,'Positions',posit,'Whisker',1,'ColorGroup',gcolor)
 
 ylim  ([0, 30])
grid on 








%  for i =1:1:num_cal
%     mean_our(i) = sum ( Loss_our(:,i))/(num_x*num_y);
%      mean_P(i) = sum ( Loss_P(:,i))/(num_x*num_y);
%  end
%  
%  plot (posit(1:2:end),mean_P,'-r');
%  plot (posit(2:2:end),mean_our,'-b');
%  
 
 
 
% for i = 1:1:num_cal
%     for j = 1:1:num_x*num_y
%         plot([i-0.15,i+0.15],[Loss_our(j,i),Loss_our(j,i)],'k');
%         
% %         plot(i*ones(1,num_x*num_y)+0.2,Loss_P(:,i),'b*');
%     end
% end
% xlim ([0 num_cal+1])
% ylim  ([0, 50])
%  

%         Loss_of_M_Method=Loss_of_P_Method(Tht,Ins_para_P_nodistortion,Points_c_final_undistortion,Points_l_final,resolution);
        
        
%          for i = 1:1:size(Points_c_final_undistortion,1)/36
%             figure()
%             plot(Points_c_final_undistortion((i-1)*36+1:i*36,1),Points_c_final_undistortion((i-1)*36+1:i*36,2),'ro','MarkerSize',5);
%             hold on
%             plot(u_x((i-1)*36+1:i*36),u_y((i-1)*36+1:i*36),'b*','MarkerSize',5);
%             axis([0 resolution(1) 0 resolution(2)]);
%             axis equal
%             
%         end
        
end





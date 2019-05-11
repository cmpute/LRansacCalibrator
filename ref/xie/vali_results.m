function []=vali_results(points_3d,HH,points_3d_dis,points_3d_int,flag)


for i = 1:1:size(points_3d,2)
 
        project_by_calib(:,i)=HH*[points_3d(:,i);1];
        
              
 
end  
project_by_calib(1,:)=project_by_calib(1,:)./project_by_calib(3,:);
project_by_calib(2,:)=project_by_calib(2,:)./project_by_calib(3,:);
project_by_calib(3,:)=project_by_calib(3,:)./project_by_calib(3,:);
dis_color = points_3d_dis/max(points_3d_dis)*255;
int_color = points_3d_int/max(points_3d_int)*255;
color_map = jet(256);
color_map=flipud(color_map);


if flag == 1
for i = 1:1:255
plot ( project_by_calib(1,(dis_color>i-1 & dis_color<i)), project_by_calib(2,(dis_color>i-1 & dis_color<i))...
                                  ,'*' ,'MarkerSize',1,'MarkerEdgeColor',color_map(i,:))
    hold on; 
end
end

if flag == 0
for i = 1:1:255
plot ( project_by_calib(1,(int_color>i-1 & int_color<i)), project_by_calib(2,(int_color>i-1 & int_color<i))...
                                  ,'*' ,'MarkerSize',1,'MarkerEdgeColor',color_map(i,:))
    hold on; 
end
end

end

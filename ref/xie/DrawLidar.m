function DrawLidar(Loc_cam,R_c2w,Loc_c2l,R_l2c)


o_l =Loc_cam-R_c2w*Loc_c2l;
o_l_x =(o_l+R_c2w*R_l2c(:,1)*2);
o_l_y =(o_l+R_c2w*R_l2c(:,2)*2);
o_l_z =(o_l+R_c2w*R_l2c(:,3)*2);

plot3([o_l(1),o_l_x(1)],[o_l(2),o_l_x(2)],[o_l(3),o_l_x(3)],'Color','k','LineWidth',1);hold on
plot3([o_l(1),o_l_y(1)],[o_l(2),o_l_y(2)],[o_l(3),o_l_y(3)],'Color','k','LineWidth',1);hold on
plot3([o_l(1),o_l_z(1)],[o_l(2),o_l_z(2)],[o_l(3),o_l_z(3)],'Color','k','LineWidth',1);hold on

size_c = 2*[0.2,0.4,0.2];
lx=size_c(1)*R_c2w*R_l2c(:,1);
ly=size_c(2)*R_c2w*R_l2c(:,2);
lz=size_c(3)*R_c2w*R_l2c(:,3);



%lidar box
VertexPoints(:,1) =  o_l-0.5*lx+0.5*ly+0.5*lz;
VertexPoints(:,2) =  VertexPoints(:,1)+lx;
VertexPoints(:,3) =  VertexPoints(:,2)-ly;
VertexPoints(:,4) =  VertexPoints(:,3)-lx;

VertexPoints(:,5:8) =VertexPoints(:,1:4)-lz  ;

plot3(VertexPoints(1,[1 2 3 4 1]),VertexPoints(2,[1 2 3 4 1]),VertexPoints(3,[1 2 3 4 1]),'-b','LineWidth',0.5);hold on
plot3(VertexPoints(1,[5 6 7 8 5]),VertexPoints(2,[5 6 7 8 5]),VertexPoints(3,[5 6 7 8 5]),'-b','LineWidth',0.5);hold on
plot3(VertexPoints(1,[1 4 8 5 1]),VertexPoints(2,[1 4 8 5 1]),VertexPoints(3,[1 4 8 5 1]),'-b','LineWidth',0.5);hold on
plot3(VertexPoints(1,[2 3 7 6 2]),VertexPoints(2,[2 3 7 6 2]),VertexPoints(3,[2 3 7 6 2]),'-b','LineWidth',0.5);hold on
plot3(VertexPoints(1,[1 2 6 5 1]),VertexPoints(2,[1 2 6 5 1]),VertexPoints(3,[1 2 6 5 1]),'-b','LineWidth',0.5);hold on

% text(o_l_x(1),o_l_x(2),o_l_x(3),'X_{Lidar}');
% text(o_l_y(1),o_l_y(2),o_l_y(3),'Y_{Lidar}');
% text(o_l_z(1),o_l_z(2),o_l_z(3),'Z_{Lidar}');




end
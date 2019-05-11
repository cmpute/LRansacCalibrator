function DrawCamera(Loc_cam,R_c2w)

o_c =Loc_cam;
o_c_x =(Loc_cam+R_c2w(:,1)*2);
o_c_y =(Loc_cam+R_c2w(:,2)*2);
o_c_z =(Loc_cam+R_c2w(:,3)*2);

plot3([o_c(1),o_c_x(1)],[o_c(2),o_c_x(2)],[o_c(3),o_c_x(3)],'Color','k','LineWidth',1);hold on
plot3([o_c(1),o_c_y(1)],[o_c(2),o_c_y(2)],[o_c(3),o_c_y(3)],'Color','k','LineWidth',1);hold on
plot3([o_c(1),o_c_z(1)],[o_c(2),o_c_z(2)],[o_c(3),o_c_z(3)],'Color','k','LineWidth',1);hold on

size_c = 2*[0.2,0.2,0.4];
lx=size_c(1)*R_c2w(:,1);
ly=size_c(2)*R_c2w(:,2);
lz=size_c(3)*R_c2w(:,3);



%camera box
VertexPoints(:,1) =  o_c-0.5*lx+0.5*ly;
VertexPoints(:,2) =  VertexPoints(:,1)+lx;
VertexPoints(:,3) =  VertexPoints(:,2)-ly;
VertexPoints(:,4) =  VertexPoints(:,3)-lx;

VertexPoints(:,5:8) =VertexPoints(:,1:4)-lz  ;

plot3(VertexPoints(1,[1 2 3 4 1]),VertexPoints(2,[1 2 3 4 1]),VertexPoints(3,[1 2 3 4 1]),'-r','LineWidth',0.5);hold on
plot3(VertexPoints(1,[5 6 7 8 5]),VertexPoints(2,[5 6 7 8 5]),VertexPoints(3,[5 6 7 8 5]),'-r','LineWidth',0.5);hold on
plot3(VertexPoints(1,[1 4 8 5 1]),VertexPoints(2,[1 4 8 5 1]),VertexPoints(3,[1 4 8 5 1]),'-r','LineWidth',0.5);hold on
plot3(VertexPoints(1,[2 3 7 6 2]),VertexPoints(2,[2 3 7 6 2]),VertexPoints(3,[2 3 7 6 2]),'-r','LineWidth',0.5);hold on
plot3(VertexPoints(1,[1 2 6 5 1]),VertexPoints(2,[1 2 6 5 1]),VertexPoints(3,[1 2 6 5 1]),'-r','LineWidth',0.5);hold on

%camera len
LenPoints(:,1:4)=VertexPoints(:,1:4)+0.5*lz;
plot3(LenPoints(1,[1 2 3 4 1]),LenPoints(2,[1 2 3 4 1]),LenPoints(3,[1 2 3 4 1]),'-r','LineWidth',0.5);hold on
for i=1:1:4
    plot3([LenPoints(1,i),o_c(1)],[LenPoints(2,i),o_c(2)],[LenPoints(3,i),o_c(3)],'-r','LineWidth',0.5);hold on
end

% text(o_c_x(1),o_c_x(2),o_c_x(2),'X_{camera}');
% text(o_c_y(1),o_c_y(2),o_c_y(3),'Y_{camera}');
% text(o_c_z(1),o_c_z(2),o_c_z(3),'Z_{camera}');
end



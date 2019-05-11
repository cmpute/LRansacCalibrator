AA= [fx,0,u0;0,fy,v0;0,0,1];
RT_P2=AA\HH;
porp = (1/(RT_P2(1,1)^2+RT_P2(2,1)^2+RT_P2(3,1)^2))^0.5;
RT_P2=RT_P2*porp;


backside = loc(1,:)>-2;

loc(:,backside)=[];


points_3d=loc(1:3,:);
points_3d_int=loc(4,:);
points_3d_dis=loc(5,:);




function R=rodrigues(Ori)

R1=[1,0,0;
    0,cos(Ori(1)),sin(Ori(1));
    0,-sin(Ori(1)),cos(Ori(1));];

R2=[cos(Ori(2)),0,-sin(Ori(2));
    0,1,0;
    sin(Ori(2)),0,cos(Ori(2));];
R3=[cos(Ori(3)),sin(Ori(3)),0;
    -sin(Ori(3)),cos(Ori(3)),0;
    0,0,1;];
R=R3*R2*R1;


end
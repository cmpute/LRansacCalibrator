function Ins_para=gener_cam(resolution,level)


    fx=1000+300*rand;
    fy=1000+300*rand;
    u0= resolution(1)/2-20+40*rand;
    v0= resolution(2)/2-20+40*rand;

    k1 = level*(1+1*rand)*1e-6;
    k2 = -level*(1+1*rand)*1e-12;
    p1 = level*(1+2*rand)*1e-6;
    p2 = -level*(1+2*rand)*1e-7;




    Ins_para = [fx,fy,u0,v0,k1,k2,p1,p2];
    
end
x = fmincon(@myfun,[-0.5,-0.5,2.5,1,0,0,0,1,0],[],[],[],[],[],[],@mycon);
dx = 0.158;
dy = 0.165;


X_ = x(4:6);
Y_ = x(7:9);
Z_ = cross ( x(4:6),x(7:9) );
orig = x(1:3)+dx*x(4:6)+dy*x(7:9);

R   = [X_',Y_',Z_'];
R_l = R';
T_l = orig;


TT = T_l' - Tc_1*0.001;
RR = Rc_1'*R;
function f =fminfun (X,L_3dpoints,Line)
    min_fun1 =fun(X,L_3dpoints{1:6},Line{1});
    min_fun2 =fun(X,L_3dpoints{7:12},Line{2});
    min_fun3 =fun(X,L_3dpoints{12:17},Line{3});
    f = (min_fun1+min_fun2+min_fun3);

end
function P = interX(x1,y1,x2,y2)

%fit linear polynomial
p1 = polyfit(x1,y1,1);
p2 = polyfit(x2,y2,1);
%calculate intersection
x_intersect = fzero(@(x) polyval(p1-p2,x),3);
y_intersect = polyval(p1,x_intersect);
P=[x_intersect;y_intersect,1];
% line(x1,y1);
% hold on;
% line(x2,y2);
% plot(x_intersect,y_intersect,'r*') 
end
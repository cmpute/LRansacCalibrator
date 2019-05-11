function filename=buling(x)
a=num2str(x);
filename=[];
for i=1:1:6-length(a)
filename=[filename,num2str(0)];
end
filename=[filename,a];
end
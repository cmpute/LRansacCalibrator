for i = 1:1:20
    for j =1:1:6
    MMethod5(i,j)=simresults5{i}{1}(j);
    OurMethod5(i,j)=simresults5{i}{2}(j);
    MMethodnoised5(i,j)=simresults5{i}{3}(j);
    end
end

for i = 1:1:20
    for j =1:1:6
    MMethod10(i,j)=simresults10{i}{1}(j);
    OurMethod10(i,j)=simresults10{i}{2}(j);
    MMethodnoised10(i,j)=simresults10{i}{3}(j);
    end
end

for i = 1:1:20
    for j =1:1:6
    MMethod20(i,j)=simresults20{i}{1}(j);
    OurMethod20(i,j)=simresults20{i}{2}(j);
    MMethodnoised20(i,j)=simresults20{i}{3}(j);
    end
end

Poses5=[];
for i =1:1:6
   Poses5 = [Poses5  MMethodnoised5(:,i) MMethod5(:,i)  OurMethod5(:,i)];
%    Poses5 = [Poses5 zeros(8,1)];
end
    
Poses10=[];
for i =1:1:6
   Poses10 = [Poses10  MMethodnoised10(:,i) MMethod10(:,i)  OurMethod10(:,i)];
end
    
Poses20=[];
for i =1:1:6
   Poses20 = [Poses20  MMethodnoised20(:,i) MMethod20(:,i)  OurMethod20(:,i)];
end





%
posit = [1 2 3 5 6 7 9 10 11 13 14 15 17 18 19 21 22 23 ];

g = 1:1:18;
gcolor = 3:1:5;

% gcolor =cell(1,3);
% gcolor{1}='r';
% gcolor{2}='g';
% gcolor{3}='b';
gcolor =[gcolor gcolor gcolor gcolor gcolor gcolor ];

label = cell(1,18);
for i=1:1:18
    label{i}='';
end

label{2}='3';
label{5}='5';
label{8}='10';
label{11}='20';
label{14}='30';
label{17}='50';



figure
boxplot(Poses5,g,'Positions',posit,'Whisker',1,'ColorGroup',gcolor,'Labels',label)
axis([0 24 0 60]);
grid on
ylabel('Reprojection error /pixel')
xlabel('Evaluation Distance /m')

figure
boxplot(Poses10,g,'Positions',posit,'Whisker',1,'ColorGroup',gcolor,'Labels',label)
axis([0 24 0 50]);
grid on
ylabel('Reprojection error /pixel')
xlabel('Evaluation Distance /m')

figure
boxplot(Poses20,g,'Positions',posit,'Whisker',1,'ColorGroup',gcolor,'Labels',label)
axis([0 24 0 25]);
grid on
ylabel('Reprojection error /pixel')
xlabel('Evaluation Distance /m')








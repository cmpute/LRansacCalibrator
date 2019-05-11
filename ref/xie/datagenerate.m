simresults5 = cell(1,20);
simresults10 = cell(1,20);
simresults20 = cell(1,20);

for i= 1:1:20
%     simresults5{i}=simulate(5);
%     simresults10{i}=simulate(10);
    simresults20{i}=simulate(20);
    step = i
end
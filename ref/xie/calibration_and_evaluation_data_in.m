%% calibration and evaluation data in
resolution = [1920,1200];
num_x = 6;
num_y = 6;

calirationDataInd = [1 3 5 7 11 13 14 38 40 ];
num_cal = length(calirationDataInd);
evaluationDataInd = [2 4 6 8 10 12 36 37 41 ];
Points_l_final_calibration = zeros (length(calirationDataInd),3);
Points_c_final_calibration = zeros (length(calirationDataInd),2);
Points_l_final_evaluation = zeros (length(evaluationDataInd),3);
Points_c_final_evaluation = zeros (length(evaluationDataInd),2);

%% Points_l_final from txt

txtPath = 'C:\Users\Administrator\Desktop\params_1\';
for i = 1:1:length(calirationDataInd)
    Points_l_final_calibration((i-1)*36+1:i*36,:)=load([txtPath,num2str(calirationDataInd(i)-1),'_features.txt']);
end
for i = 1:1:length(evaluationDataInd)
    Points_l_final_evaluation((i-1)*36+1:i*36,:)=load([txtPath,num2str(evaluationDataInd(i)-1),'_features.txt']);
end
Points_l_final=Points_l_final_calibration;
% Points_l_final=Points_l_final_evaluation;


%% Points_c_final from mat
 tmpStruct = load("paper_points_c_final_all.mat");
 paper_points_c_final_all = tmpStruct.Points_c_final;
 
 for i = 1:1:length(calirationDataInd)
    Points_c_final_calibration((i-1)*36+1:i*36,:)=paper_points_c_final_all((calirationDataInd(i)-1)*36+1:calirationDataInd(i)*36,:);
 end
for i = 1:1:length(evaluationDataInd)
    Points_c_final_evaluation((i-1)*36+1:i*36,:)=paper_points_c_final_all((evaluationDataInd(i)-1)*36+1:evaluationDataInd(i)*36,:);
end
Points_c_final=Points_c_final_calibration;
% Points_c_final=Points_c_final_evaluation; 



%% Camera data in
load('CALIB(1).mat');
fx = cameraParams.FocalLength(1);
fy = cameraParams.FocalLength(2);
u0 = cameraParams.PrincipalPoint(1);
v0 = cameraParams.PrincipalPoint(2);
k1 = cameraParams.RadialDistortion(1);
k2 = cameraParams.RadialDistortion(2);
p1 = cameraParams.TangentialDistortion(1);
p2 = cameraParams.TangentialDistortion(2);
Ins_para = [fx,fy,u0,v0,k1,k2,p1,p2];
%% 


%Calibration LiDAR and Camera
%The Input are the image forder and results of 3d detection txt

folderin = '/media/jacobz/Library/MCity/extri_data/extri_bags_0505/all_3_fixed/';
txtPath = '/media/jacobz/Library/MCity/extri_data/extri_bags_0505/all_3_fixed/';
%% calibration and evaluation data in
resolution = [1920,1080];
num_x = 6;
num_y = 6;

calirationDataInd = [1 3 5 7 11 13 14 38 40 ];
num_cal = length(calirationDataInd);

Points_l_final_calibration = zeros (length(calirationDataInd),3);
Points_c_final_calibration = zeros (length(calirationDataInd),2);





%% Points_l_final from forder
folderout = [folderin 'rename\'];
files = dir([folderin '\*.jpg']);
for i = 1:1:numel(files)
    oldname = files(i).name;
    I = imread([folderin,oldname]);
    %[pathstr, name, ext] = fileparts(oldname) ;
        newname = strcat(buling(i),'.jpg');
        imwrite(I,[folderout,newname],'jpg');
end

%%

Points_c_final=[];
j=1;
i=1;
 files = dir([folderin '*.png']);
for i =1:1:num_cal
    %%

    oldname = files(i).name;
    X = imread([folderin,oldname]);
    % ��ȡͼ��
    grayThreshod=50;

    flag_usingBin = 1;
    if (flag_usingBin)
        flag = 1;
        while (flag)
            f = rgb2gray(X);
            %%%%%%%%%%%%%%%%%��ֵ��%%%%%%%%%%%%%%%%%%%
            blackPosition = f < grayThreshod;
            whitePositon = ~blackPosition;
            f(whitePositon)=255;


            ori_im = double(f) / 255;                   %unit8ת��Ϊ64Ϊ˫����double64

            height = size(ori_im, 1);
            width = size(ori_im, 2);



            [CornerPoint,boardSize]=detectCheckerboardPoints(f);
            if size(CornerPoint,1)==36
                flag=0;
            end
            grayThreshod = grayThreshod +1


        end
    else
            f = rgb2gray(X);
        

            ori_im = double(f) / 255;                   %unit8ת��Ϊ64Ϊ˫����double64

            height = size(ori_im, 1);
            width = size(ori_im, 2);



            [CornerPoint,boardSize]=detectCheckerboardPoints(f);
    end




    % figure()
    % imshow (f);

    figure()
    hold off;
    imshow(X);
    hold on;


    CornerPoint_int = ceil(CornerPoint);
    CornerImage =  zeros(height, width);
    CornerImage(CornerPoint_int(:,2)+(CornerPoint_int(:,1)-1)*height)=1;
    plot(CornerPoint(:,1),CornerPoint(:,2),'r*','MarkerSize',2)
    hold on

    CornerPointTmp = [];

    for jf =1:1:6

        CornerIndex(1)=find (CornerPoint(:,1)==min(CornerPoint(:,1)));
        tempIndex = find (CornerPoint(:,2)<CornerPoint(CornerIndex(1),2));

        for ii = 2:1:6
            CornerIndex(ii)=tempIndex(CornerPoint(tempIndex,1)==min(CornerPoint(tempIndex,1)));
            tempIndex = find (CornerPoint(:,2)<CornerPoint(CornerIndex(ii),2));
        end
        CornerPointTmp(6*(jf-1)+1:6*jf,:)=CornerPoint( CornerIndex,:);
        CornerPoint( CornerIndex,:)=[];
    end

    CornerPoint =  CornerPointTmp;


    flag2=input('right?:');
    %         flag2 =1;
    if flag2
        Points_c_final(36*(i-1)+1:36*i,:) = CornerPoint;


    else
        calirationDataInd(i)=[];
        num_cal = num_cal-1;
    end





end

%% Points_l_final from txt


for i = 1:1:length(calirationDataInd)
    Points_l_final_calibration((i-1)*36+1:i*36,:)=load([txtPath,num2str(calirationDataInd(i)-1),'_features.txt']);
end

Points_l_final=Points_l_final_calibration;

%% calibration
[HH,H]=solv_the_initial_H_4(Points_c_final,Points_l_final);


initial_H = H(1:12);
%fine tune without H
initial_P = [resolution(1)/2;resolution(2)/2;0;0;0;0];
proportion = ones(6,1);

min_fun=@(P)FinetuneFunWithoutH(initial_H,P,Points_c_final,Points_l_final,resolution,1,proportion);
initial_P = fminsearch(min_fun,initial_P,optimset('MaxFunEvals',40000,'MaxIter',40000,...
                           'Algorithm','levenberg-marquardt','ToLX',1e-6));%'Display','iter',
                       
%��ÿ���������н��ƹ�һ��                       
proportion=initial_P;
min_fun=@(P)FinetuneFunWithoutH(initial_H,P,Points_c_final,Points_l_final,resolution,1,proportion);
initial_P = fminsearch(min_fun,ones(6,1),optimset('MaxFunEvals',40000,'MaxIter',40000,...
                            'Algorithm','levenberg-marquardt','ToLX',1e-6));%'Display','iter',
% P=initial_P.*proportion;


proportion_ =[initial_H;proportion.*initial_P];
min_fun = @(P) FinetuneFun(P,Points_c_final,Points_l_final,resolution,1,proportion_);
loss_min = 10000;
rand_value = ones(18,1);
for i=1:1:30
   
    Calib_Re = fminsearch(min_fun,rand_value,optimset('MaxFunEvals',40000,'MaxIter',40000,...
                            'Algorithm','levenberg-marquardt','ToLX',1e-6));%'Display','iter',
    
                        
%       loss = Loss_of_Our_Method(Calib_Re,Points_c_final,Points_l_final,resolution,0);
                
                        
                        
      loss = FinetuneFun(Calib_Re,Points_c_final,Points_l_final,resolution,0,proportion_);
    if loss<loss_min
        loss_min=loss;
        Calib_Re_final = Calib_Re;
    end
     
     rand_value = (0.8+0.4*rand(18,1)).*Calib_Re_final;  
     i            
end
Calib_Re = Calib_Re_final.*proportion_;
%% evaluate

 HH = [Calib_Re(1:4)';Calib_Re(5:8)';Calib_Re(9:12)'];
 
 u0 = Calib_Re(13);
 v0 = Calib_Re(14);
 
 k1 = Calib_Re(15);
 k2 = Calib_Re(16);
 
 p1 = Calib_Re(17);
 p2 = Calib_Re(18);
 
 ProjectPoints = HH *[Points_l_final';ones(1,size(Points_l_final',2))];
 ProjectPoints(1,:)=ProjectPoints(1,:)./ProjectPoints(3,:);
 ProjectPoints(2,:)=ProjectPoints(2,:)./ProjectPoints(3,:);
 ProjectPoints(3,:)=ProjectPoints(3,:)./ProjectPoints(3,:);
 

  
  u_x=ProjectPoints(1,:)-u0;  %ת���������
  u_y=ProjectPoints(2,:)-v0;  
 

        %���ǻ���
        r_2 = u_x.^2+u_y.^2;
        u_x_ = u_x.*(1+k1*r_2+k2*r_2.^2)+(2*p1*u_x.*u_y+p2*(r_2+2*u_x.^2));
        u_y_ = u_y.*(1+k1*r_2+k2*r_2.^2)+(2*p2*u_x.*u_y+p1*(r_2+2*u_y.^2));
        
        u_x = u_x_ +u0;   %ת����������
        u_y = u_y_ +v0;
        Loss_our = zeros(num_x*num_y,num_cal);
    
        
        
        m=1;
        for i =1:1:num_cal
            for j =1:1:num_x*num_y
                Loss_our(j,i) = ((Points_c_final(m,1)-u_x(m))^2+(Points_c_final(m,2)-u_y(m))^2)^0.5;
                m=m+1;
            end 
        end
        
        
            mean_our = sum(sum(Loss_our))/(num_cal*num_x*num_y);
            
            Loss_our_ = reshape(Loss_our,1,num_cal*num_x*num_y);
          
            
            outLiers_our = Loss_our_>2*mean_our;
   
            
            Loss_our_(outLiers_our)=[];
        
        
            RMS_our = (sum(Loss_our_.^2)/length(Loss_our_))^0.5;
        




        for i = 1:1:num_cal
            oldname = files(i).name;
            X = imread([folderin,oldname]);
            figure()
            imshow(X);
            plot(Points_c_final((i-1)*36+1:i*36,1),Points_c_final((i-1)*36+1:i*36,2),'ro','MarkerSize',5);
            hold on
            plot(u_x((i-1)*36+1:i*36),u_y((i-1)*36+1:i*36),'w*','MarkerSize',5);
            axis([0 resolution(1) 0 resolution(2)]);
            axis equal
              
        end


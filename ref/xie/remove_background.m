[filename,pathname]=uigetfile('*.jpg','选择数据文件');
X1 = imread([pathname filename]);     % 读取图像
[filename,pathname]=uigetfile('*.jpg','选择数据文件');
X2 = imread([pathname filename]);     % 读取图像

X1=histeq(X1);
X2=histeq(X2);

X3(:,:,1)= [X2(:,1:600,1),X1(:,601:end,1)];
X3(:,:,2)= [X2(:,1:600,2),X1(:,601:end,2)];
X3(:,:,3)= [X2(:,1:600,3),X1(:,601:end,3)];
imshow(X3);

X4=X;
tempimg=zeros(size(X,1),size(X,2),3);
for ii=1:1:3
    tempimg(:,:,ii)=double(X4(:,:,ii))-double(X3(:,:,ii));
end
for ii=1:1:size(tempimg,1)
    for jj =1:1:size(tempimg,2)
        if (tempimg(ii,jj,1)^2+tempimg(ii,jj,2)^2+tempimg(ii,jj,3)^2<=200)
            X4(ii,jj,:)=255;
        end
    end
end


for ii=1:1:size(X,1)
    for jj =1:1:size(X,2)
        temp=double(X(ii,jj,:))-double(X3(ii,jj,:));
        temp=reshape(temp,1,3);
        if (norm(temp,2)<=5)
            X4(ii,jj,:)=255;
        end
    end
end
    

function EagePointImage_out = EagePoint(image_gin_)

%% ÌáÈ¡±ßÔµµã
FilterMode = [ 1 1 1 0  -1 -1 -1 ;
 1 1 1 0  -1 -1 -1 ;
 1 1 1 0  -1 -1 -1 ;];
EagePointImage = imfilter(image_gin_,FilterMode,'replicate');
EagePointImage = abs(EagePointImage);
EagePointImage = EagePointImage/max(max(EagePointImage));
% imshow(EagePointImage);
EagePoint = EagePointImage>0.3 & image_gin_>0.5;
EagePointImage_out = zeros(size(EagePointImage));
EagePointImage_out(EagePoint)=1;
% imshow(EagePointImage_out);

[yy,xx] = find(EagePointImage_out==1);



% imshow(image_gin_);
% hold on
plot(xx,yy,'r*');
hold on
end
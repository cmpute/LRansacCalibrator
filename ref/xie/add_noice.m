function [CORNER_POINTS,PROJECTION_POINTS]=add_noice(CORNER_POINTS,PROJECTION_POINTS,level)   

    num = length(CORNER_POINTS);
    num_corner = size(CORNER_POINTS{1},2);
    mu_l = [0,0,0]; %mean error of lidar
    mu_c = [0,0]; %mean error of cam
    sigma_l = 0.0005*level*[1 0.5 0.5; 0.5 1 0.5;0.5 0.5 1];
    sigma_c = 1*level*[1 0.5; 0.5 1];
    
    R_l = chol(sigma_l);
    R_c = chol(sigma_c);

for i= 1:1:num
    z_l = (repmat(mu_l,num_corner,1) + randn(num_corner,3)*R_l)';
    z_c = (repmat(mu_c,num_corner,1) + randn(num_corner,2)*R_c)';
    CORNER_POINTS{i}=CORNER_POINTS{i}+z_l;
    PROJECTION_POINTS{i}(1:2,:)=PROJECTION_POINTS{i}(1:2,:)+z_c;
end




end
function [Loc_par,R_w2p,R_p2w,X_,Y_,Z_,corner_point_ ]=gener_par(num_x,num_y,ddx,ddy,varargin)
    if nargin==4
        Loc_par = [-7+14*rand;-2+3*rand;1+15*rand];
    else 
        Loc_par = varargin{1};
    end
    Ori_par = [0.5*rand;0.5*rand;rand];
    R_w2p   = rodrigues(Ori_par);
    R_p2w   = R_w2p';
    X_ = R_p2w(:,1);
    Y_ = R_p2w(:,2);
    Z_ = R_p2w(:,3);
    
%     corner_point = cell (num_x,num_y);
    corner_point_ = zeros(3,num_x*num_y);
%     corner_point{1,1} = Loc_par;
    corner_point_(:,1)=Loc_par;
    ii=1;
    for i = 1:1:num_x
        for j=1:1:num_y
            if (i==1 && j==1)
                ii=ii+1;
                continue;
            end
%             corner_point{i,j} = corner_point{1,1} + (i-1)*ddx*X_+(j-1)*ddy*Y_;
            corner_point_(:,ii)=Loc_par + (i-1)*ddx*X_+(j-1)*ddy*Y_;
            ii=ii+1;
        end
    end



end

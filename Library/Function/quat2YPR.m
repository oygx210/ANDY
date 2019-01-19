function [YPR] = quat2YPR(inQuat)
%QUAT2YPR Summary of this function goes here
%   Detailed explanation goes here
    qw=inQuat(1);
    qx=inQuat(2);
    qy=inQuat(3);
    qz=inQuat(4);

    YPR = [ atan2( 2*(qx.*qy+qw.*qz), qw.^2 + qx.^2 - qy.^2 - qz.^2 ); 
        asin( -2*(qx.*qz-qw.*qy) );
        atan2( 2*(qy.*qz+qw.*qx), qw.^2 - qx.^2 - qy.^2 + qz.^2 );];
end


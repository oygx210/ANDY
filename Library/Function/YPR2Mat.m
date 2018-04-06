function RPYMat=YPR2Mat(yaw,pitch,roll)
% RPYMat=YPR2Mat(yaw,pitch,roll)
% Convert Yaw <yaw>, Pitch <pitch>, Roll <roll> to rotation matrix [RPYMat] 
% (from parent frame to child frame).

% Copyright 2018 Robotics & Mechatronics Lab, Virginia Tech
% $Author: Jiamin Wang $  $Date: 2018/3/18 $ $Revision: 1.0 $

    YawMat=[cos(yaw) sin(yaw) 0;-sin(yaw) cos(yaw) 0;0 0 1;];
    PitchMat=[cos(pitch) 0 -sin(pitch);0 1 0;sin(pitch) 0 cos(pitch);];
    RollMat=[1 0 0; 0 cos(roll) sin(roll); 0 -sin(roll) cos(roll);];
    RPYMat=RollMat*PitchMat*YawMat;
end
